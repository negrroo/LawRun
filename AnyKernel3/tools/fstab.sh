#!/sbin/sh

LD_PATH=/system/lib
if [ -d /system/lib64 ]; then
  LD_PATH=/system/lib64
fi

exec_util() {
  LD_LIBRARY_PATH=/system/lib64 $UTILS $1
}

# Add fstab with f2fs mount points and encryptable flags
ui_print " "; ui_print "Add modified fstab with f2fs mount points..."; ui_print " ";
umount /vendor || true
mount -o rw /dev/block/bootdevice/by-name/vendor /vendor
chmod -R 644 /tmp/anykernel/ramdisk/fstab.qcom;
exec_util "cp -a /tmp/anykernel/ramdisk/fstab.qcom /vendor/etc/"
rm $ramdisk/fstab.qcom
umount /vendor || true
