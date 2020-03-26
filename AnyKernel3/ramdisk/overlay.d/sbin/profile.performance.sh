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
############################LawRun-Performance##################################

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
echo "schedutil" > /sys/devices/system/cpu/cpu4/cpufreq/scaling_governor
echo "825600" > /sys/devices/system/cpu/cpu4/cpufreq/scaling_min_freq
echo "2803200" > /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq

# BOost
echo "1516800" > /sys/module/cpu_input_boost/parameters/input_boost_freq_lp
echo "1363200" > /sys/module/cpu_input_boost/parameters/input_boost_freq_hp
echo "125" > /sys/module/cpu_input_boost/parameters/input_boost_duration

# Dynamic Schedtune Boost
echo "2000" > /sys/module/cpu_input_boost/parameters/dynamic_stune_boost_duration
echo "50" > /sys/module/cpu_input_boost/parameters/dynamic_stune_boost

# SILVER Cluster Limiter
echo "0" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/down_rate_limit_us
echo "1228800" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/hispeed_freq
echo "90" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/hispeed_load
echo "0" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/iowait_boost_enable
echo "1" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/pl
echo "0" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/up_rate_limit_us

# GOLD Cluster Limiter
echo "0" > /sys/devices/system/cpu/cpu4/cpufreq/schedutil/down_rate_limit_us
echo "1536000" > /sys/devices/system/cpu/cpu4/cpufreq/schedutil/hispeed_freq
echo "90" > /sys/devices/system/cpu/cpu4/cpufreq/schedutil/hispeed_load
echo "0" > /sys/devices/system/cpu/cpu4/cpufreq/schedutil/iowait_boost_enable
echo "1" > /sys/devices/system/cpu/cpu4/cpufreq/schedutil/pl
echo "0" > /sys/devices/system/cpu/cpu4/cpufreq/schedutil/up_rate_limit_us

################################################################################

                          #####################
                          #                   #
                          #        GPU        #
                          #                   #
                          #####################

################################################################################

# Low Freq
echo "257000000" > /sys/class/kgsl/kgsl-3d0/devfreq/min_freq 
echo "257000000" > /sys/class/kgsl/kgsl-3d0/min_gpuclk

# High Freq
echo "820000000" > /sys/class/kgsl/kgsl-3d0/devfreq/max_freq 
echo "820000000" > /sys/class/kgsl/kgsl-3d0/max_gpuclk

# Limiter
echo "0" > /sys/class/kgsl/kgsl-3d0/throttling
echo "7" > /sys/class/kgsl/kgsl-3d0/default_pwrlevel
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
chmod 664 /sys/class/thermal/thermal_message/sconfig
echo 10 > /sys/class/thermal/thermal_message/sconfig
chmod 644 /sys/class/thermal/thermal_message/sconfig

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
echo "2048,3072,11520,16640,24320,42240" > /sys/module/lowmemorykiller/parameters/minfree

# zRam
swapoff /dev/block/zram0
echo 1 > /sys/block/zram0/reset
echo 0 > /sys/block/zram0/disksize
echo 536870912 > /sys/block/zram0/disksize
echo 512M > /sys/block/zram0/mem_limit
echo 8 > /sys/block/zram0/max_comp_streams
mkswap /dev/block/zram0
swapon /dev/block/zram0 -p 32758
echo 65 > /proc/sys/vm/swappiness
echo 10 > /proc/sys/vm/dirty_background_ratio
echo 60 > /proc/sys/vm/vfs_cache_pressure
echo 3000 > /proc/sys/vm/dirty_writeback_centisecs

# Profile Log
dt=`date '+%d/%m/%Y %H:%M:%S'`
echo "$dt Performance LRK-P applied" >> /storage/emulated/0/LawRun-Kernel/log.txt

################################LawRun-END#######################################

#start perfd back
#start perfd

## End Profile
