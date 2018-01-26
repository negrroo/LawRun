/* Copyright (c) 2015-2018, 2020 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/dma-buf.h>
#include <linux/msm_dma_iommu_mapping.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <asm/barrier.h>

struct msm_iommu_map {
	struct device *dev;
	struct msm_iommu_data *data;
	struct list_head data_node;
	struct list_head dev_node;
	struct scatterlist *sgl;
	unsigned int nents;
	enum dma_data_direction dir;
	int nents;
	int refcount;
};

static struct msm_iommu_map *msm_iommu_map_lookup(struct msm_iommu_data *data,
						  struct device *dev)
{
	struct msm_iommu_map *map;

	list_for_each_entry(map, &data->map_list, data_node) {
		if (map->dev == dev)
			return map;
	}

	return NULL;
}

static void msm_iommu_map_free(struct msm_iommu_map *map)
{
	list_del(&map->data_node);
	list_del(&map->dev_node);
	dma_unmap_sg(map->dev, &map->sg, map->nents, map->dir);
	kfree(map);
}

int msm_dma_map_sg_attrs(struct device *dev, struct scatterlist *sg, int nents,
			 enum dma_data_direction dir, struct dma_buf *dmabuf,
			 unsigned long attrs)
{
	int not_lazy = (attrs & DMA_ATTR_NO_DELAYED_UNMAP);
	struct msm_iommu_data *data = dmabuf->priv;
	struct msm_iommu_map *map;

	mutex_lock(&dev->iommu_map_lock);
	mutex_lock(&data->lock);
	map = msm_iommu_map_lookup(data, dev);
	if (map) {
		map->refcount++;
		sg->dma_address = map->sg.dma_address;
		sg->dma_length = map->sg.dma_length;
		if (is_device_dma_coherent(dev))
			dmb(ish);
	} else {
		nents = dma_map_sg_attrs(dev, sg, nents, dir, attrs);
		if (nents) {
			map = kmalloc(sizeof(*map), GFP_KERNEL | __GFP_NOFAIL);
			map->data = data;
			map->dev = dev;
			map->dir = dir;
			map->nents = nents;
			map->refcount = 2 - not_lazy;
			map->sg.dma_address = sg->dma_address;
			map->sg.dma_length = sg->dma_length;
			list_add(&map->data_node, &data->map_list);
			list_add(&map->dev_node, &dev->iommu_map_list);
		}
	}
	mutex_unlock(&data->lock);
	mutex_unlock(&dev->iommu_map_lock);

	return nents;

static void msm_iommu_meta_put(struct msm_iommu_meta *meta);

static struct scatterlist *clone_sgl(struct scatterlist *sg, int nents)
{
	struct scatterlist *next, *s;
	int i;
	struct sg_table table;

	if (sg_alloc_table(&table, nents, GFP_KERNEL))
		return NULL;
	next = table.sgl;
	for_each_sg(sg, s, nents, i) {
		*next = *s;
		next = sg_next(next);
	}
	return table.sgl;
}

static inline int __msm_dma_map_sg(struct device *dev, struct scatterlist *sg,
				   int nents, enum dma_data_direction dir,
				   struct dma_buf *dma_buf,
				   unsigned long attrs)
{
	struct msm_iommu_map *iommu_map;
	struct msm_iommu_meta *iommu_meta = NULL;
	int ret = 0;
	bool extra_meta_ref_taken = false;
	int late_unmap = !(attrs & DMA_ATTR_NO_DELAYED_UNMAP);

	mutex_lock(&msm_iommu_map_mutex);
	iommu_meta = msm_iommu_meta_lookup(dma_buf->priv);

	if (!iommu_meta) {
		iommu_meta = msm_iommu_meta_create(dma_buf);

		if (IS_ERR(iommu_meta)) {
			mutex_unlock(&msm_iommu_map_mutex);
			ret = PTR_ERR(iommu_meta);
			goto out;
		}
		if (late_unmap) {
			kref_get(&iommu_meta->ref);
			extra_meta_ref_taken = true;
		}
	} else {
		kref_get(&iommu_meta->ref);
	}

	mutex_unlock(&msm_iommu_map_mutex);

	mutex_lock(&iommu_meta->lock);
	iommu_map = msm_iommu_lookup(iommu_meta, dev);
	if (!iommu_map) {
		iommu_map = kmalloc(sizeof(*iommu_map), GFP_KERNEL);

		if (!iommu_map) {
			ret = -ENOMEM;
			goto out_unlock;
		}

		ret = dma_map_sg_attrs(dev, sg, nents, dir, attrs);
		if (!ret) {
			kfree(iommu_map);
			goto out_unlock;
		}

		iommu_map->sgl = clone_sgl(sg, nents);
		if (!iommu_map->sgl) {
			kfree(iommu_map);
			ret = -ENOMEM;
			goto out_unlock;
		}
		iommu_map->nents = nents;
		iommu_map->dev = dev;

		kref_init(&iommu_map->ref);
		if (late_unmap)
			kref_get(&iommu_map->ref);
		iommu_map->meta = iommu_meta;
		iommu_map->dir = dir;
		iommu_map->map_attrs = attrs;
		iommu_map->buf_start_addr = sg_phys(sg);
		msm_iommu_add(iommu_meta, iommu_map);

	} else {
		if (nents == iommu_map->nents &&
		    dir == iommu_map->dir &&
		    attrs == iommu_map->map_attrs &&
		    sg_phys(sg) == iommu_map->buf_start_addr) {
			sg->dma_address = iommu_map->sgl->dma_address;
			sg->dma_length = iommu_map->sgl->dma_length;

			kref_get(&iommu_map->ref);
			if (is_device_dma_coherent(dev))
				/*
				 * Ensure all outstanding changes for coherent
				 * buffers are applied to the cache before any
				 * DMA occurs.
				 */
				dmb(ish);
			ret = nents;
		} else {
			bool start_diff = (sg_phys(sg) !=
					   iommu_map->buf_start_addr);

			dev_err(dev, "lazy map request differs:\n"
				"req dir:%d, original dir:%d\n"
				"req nents:%d, original nents:%d\n"
				"req map attrs:%lu, original map attrs:%lu\n"
				"req buffer start address differs:%d\n",
				dir, iommu_map->dir, nents,
				iommu_map->nents, attrs, iommu_map->map_attrs,
				start_diff);
			ret = -EINVAL;
		}
	}
	mutex_unlock(&iommu_meta->lock);
	return ret;

out_unlock:
	mutex_unlock(&iommu_meta->lock);
out:
	if (!IS_ERR(iommu_meta)) {
		if (extra_meta_ref_taken)
			msm_iommu_meta_put(iommu_meta);
		msm_iommu_meta_put(iommu_meta);
	}
	return ret;

}

/*
 * We are not taking a reference to the dma_buf here. It is expected that
 * clients hold reference to the dma_buf until they are done with mapping and
 * unmapping.
 */
int msm_dma_map_sg_attrs(struct device *dev, struct scatterlist *sg, int nents,
		   enum dma_data_direction dir, struct dma_buf *dma_buf,
		   unsigned long attrs)
{
	int ret;

	if (IS_ERR_OR_NULL(dev)) {
		pr_err("%s: dev pointer is invalid\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(sg)) {
		pr_err("%s: sg table pointer is invalid\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(dma_buf)) {
		pr_err("%s: dma_buf pointer is invalid\n", __func__);
		return -EINVAL;
	}

	ret = __msm_dma_map_sg(dev, sg, nents, dir, dma_buf, attrs);

	return ret;
}
EXPORT_SYMBOL(msm_dma_map_sg_attrs);

static void msm_iommu_meta_destroy(struct kref *kref)
{
	struct msm_iommu_meta *meta = container_of(kref, struct msm_iommu_meta,
						ref);

	if (!list_empty(&meta->iommu_maps)) {
		WARN(1, "%s: DMA Buffer %p being destroyed with outstanding iommu mappins!\n",
		     __func__, meta->buffer);
	}
	rb_erase(&meta->node, &iommu_root);
	kfree(meta);
}

static void msm_iommu_meta_put(struct msm_iommu_meta *meta)
{
	/*
	 * Need to lock here to prevent race against map/unmap
	 */
	mutex_lock(&msm_iommu_map_mutex);
	kref_put(&meta->ref, msm_iommu_meta_destroy);
	mutex_unlock(&msm_iommu_map_mutex);
}

static void msm_iommu_map_release(struct kref *kref)
{
	struct msm_iommu_map *map = container_of(kref, struct msm_iommu_map,
						ref);
	struct sg_table table;

	table.nents = table.orig_nents = map->nents;
	table.sgl = map->sgl;
	list_del(&map->lnode);

	dma_unmap_sg(map->dev, map->sgl, map->nents, map->dir);
	sg_free_table(&table);
	kfree(map);
}

void msm_dma_unmap_sg(struct device *dev, struct scatterlist *sg, int nents,
		      enum dma_data_direction dir, struct dma_buf *dmabuf)
{
	struct msm_iommu_data *data = dmabuf->priv;
	struct msm_iommu_map *map;

	mutex_lock(&dev->iommu_map_lock);
	mutex_lock(&data->lock);
	map = msm_iommu_map_lookup(data, dev);
	if (map && !--map->refcount)
		msm_iommu_map_free(map);
	mutex_unlock(&data->lock);
	mutex_unlock(&dev->iommu_map_lock);
}

void msm_dma_unmap_all_for_dev(struct device *dev)
{
	struct msm_iommu_map *map, *tmp;

	mutex_lock(&dev->iommu_map_lock);
	list_for_each_entry_safe(map, tmp, &dev->iommu_map_list, dev_node) {
		struct msm_iommu_data *data = map->data;

		mutex_lock(&data->lock);
		msm_iommu_map_free(map);
		mutex_unlock(&data->lock);
	}
	mutex_unlock(&dev->iommu_map_lock);
}

void msm_dma_buf_freed(struct msm_iommu_data *data)
{
	struct msm_iommu_map *map, *tmp;
	int retry = 0;

	do {
		mutex_lock(&data->lock);
		list_for_each_entry_safe(map, tmp, &data->map_list, data_node) {
			struct device *dev = map->dev;

			if (!mutex_trylock(&dev->iommu_map_lock)) {
				retry = 1;
				break;
			}

			msm_iommu_map_free(map);
			mutex_unlock(&dev->iommu_map_lock);
		}
		mutex_unlock(&data->lock);
	} while (retry--);
}
