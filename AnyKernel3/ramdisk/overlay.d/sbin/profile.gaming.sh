#!/system/bin/sh

## Start Profile

#Stop perfd and remove its default values
#stop perfd
#rm /data/vendor/perfd/default_values

################################################################################
            #=================================================#
            #        **          ******      negrroo          #
            #        **          *    *      **   **          #
            #        **          ******      **  **           #
            #        **          **          *****            #
            #        *******     ** **       **  **           #
            #        *******     **   **     **   **          #
            #=================================================#
##############################LawRun-Balanced###################################

# Profile Log
dt=`date '+%d/%m/%Y %H:%M:%S'`
echo "$dt Gaming LRK applied" >> /storage/emulated/0/LawRun-Kernel/log.txt

################################################################################

                          #####################
                          #                   #
                          #        CPU        #
                          #                   #
                          #####################

################################################################################

# SILVER Cluster
echo "schedutil" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo "300000" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
echo "1766400" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq

# GOLD Cluster
echo "PIXEL_SMURFUTIL" > /sys/devices/system/cpu/cpu4/cpufreq/scaling_governor
echo "825600" > /sys/devices/system/cpu/cpu4/cpufreq/scaling_min_freq
echo "2553000" > /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq

# BOost
echo "1516800" > /sys/module/cpu_input_boost/parameters/input_boost_freq_lp
echo "1363200" > /sys/module/cpu_input_boost/parameters/input_boost_freq_hp
echo "125" > /sys/module/cpu_input_boost/parameters/input_boost_duration

# Dynamic Schedtune Boost
echo "2000" > /sys/module/cpu_input_boost/parameters/dynamic_stune_boost_duration
echo "50" > /sys/module/cpu_input_boost/parameters/dynamic_stune_boost

# SILVER Cluster Limiter
echo "20000" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/down_rate_limit_us
echo "1228800" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/hispeed_freq
echo "90" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/hispeed_load
echo "0" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/iowait_boost_enable
echo "1" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/pl
echo "1000" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/up_rate_limit_us

# GOLD Cluster Limiter
echo "20000" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/down_rate_limit_us
echo "1536000" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/hispeed_freq
echo "90" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/hispeed_load
echo "0" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/iowait_boost_enable
echo "1" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/pl
echo "1000" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/up_rate_limit_us

################################################################################

                          #####################
                          #                   #
                          #        GPU        #
                          #                   #
                          #####################

################################################################################

# Low Freq
echo "342000000" > /sys/class/kgsl/kgsl-3d0/devfreq/min_freq 
echo "342000000" > /sys/class/kgsl/kgsl-3d0/min_gpuclk

# High Freq
echo "820000000" > /sys/class/kgsl/kgsl-3d0/devfreq/max_freq 
echo "820000000" > /sys/class/kgsl/kgsl-3d0/max_gpuclk

# Limiter
echo "0" > /sys/class/kgsl/kgsl-3d0/throttling
echo "6" > /sys/class/kgsl/kgsl-3d0/default_pwrlevel
echo "msm-adreno-tz" > /sys/class/kgsl/kgsl-3d0/devfreq/governor

################################################################################

                          #####################
                          #                   #
                          #    Schedulers     #
                          #                   #
                          #####################

################################################################################

# IO Scheduler
echo "deadline" > /sys/block/sda/queue/scheduler
echo "deadline" > /sys/block/sdb/queue/scheduler
echo "deadline" > /sys/block/sdc/queue/scheduler
echo "deadline" > /sys/block/sdd/queue/scheduler
echo "deadline" > /sys/block/sde/queue/scheduler
echo "deadline" > /sys/block/sdf/queue/scheduler

################################################################################

                          #####################
                          #                   #
                          #      Battery      #
                          #                   #
                          #####################

################################################################################

# Charging mode
echo "2800000" > /sys/class/power_supply/battery/constant_charge_current_max

# Power
echo "N" > /sys/module/workqueue/parameters/power_efficient

# Thermals
echo "10" > /sys/class/thermal/thermal_message/sconfig

# Scale down in low write load
# That change tried to fix a problem for clock scaling during write requests.
# The default value for it is "0" (favor for downscale).
# For users who want performance over power they should set it to "1" (favor for upscale)
write /sys/class/mmc_host/mmc0/clk_scaling/scale_down_in_low_wr_load 1

# Panel Backlight
write /sys/class/leds/lcd-backlight/max_brightness 255

################################################################################

                          #####################
                          #                   #
                          #       Extra       #
                          #                   #
                          #####################

################################################################################

# Ram Management
# 2048 x 4 /1024 = 8
# lower number * higher multitasking
echo "0" > /sys/module/lowmemorykiller/parameters/enable_adaptive_lmk
echo "0" > /sys/module/lowmemorykiller/parameters/lmk_fast_run
echo "1" > /sys/module/lowmemorykiller/parameters/oom_reaper
echo "33792,45056,56320,123904,168960,180224" > /sys/module/lowmemorykiller/parameters/minfree

# zRam
swapoff /dev/block/zram0
echo 1 > /sys/block/zram0/reset
echo 0 > /sys/block/zram0/disksize
echo 1073741824 > /sys/block/zram0/disksize
echo 1024M > /sys/block/zram0/mem_limit
echo 8 > /sys/block/zram0/max_comp_streams
mkswap /dev/block/zram0
swapon /dev/block/zram0 -p 32758
echo 65 > /proc/sys/vm/swappiness
echo 10 > /proc/sys/vm/dirty_background_ratio
echo 60 > /proc/sys/vm/vfs_cache_pressure
echo 3000 > /proc/sys/vm/dirty_writeback_centisecs

################################LawRun-END#######################################

#start perfd back
#start perfd

## End Profile
