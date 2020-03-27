#!/system/bin/sh

################################################################################
# helper functions to allow Android init like script

function write() {
    echo -n $2 > $1
}

function copy() {
    cat $1 > $2
}

# macro to write pids to system-background cpuset
function writepid_sbg() {
    until [ ! "$1" ]; do
        echo -n $1 > /dev/cpuset/system-background/tasks;
        shift;
    done;
}

################################################################################

{

sleep 10;

################################################################################
            #=================================================#
            #        **          ******      negrroo          #
            #        **          *    *      **   **          #
            #        **          ******      **  **           #
            #        **          **          *****            #
            #        *******     ** **       **  **           #
            #        *******     **   **     **   **          #
            #=================================================#
#############################LawRun-Initation###################################

    chmod 0664 /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
    chmod 0664 /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq
    chmod 0664 /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
    chmod 0664 /sys/devices/system/cpu/cpu4/cpufreq/scaling_min_freq
    chmod 0664 /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
    chmod 0664 /sys/devices/system/cpu/cpu4/cpufreq/scaling_governor

    chmod 0664 /sys/class/kgsl/kgsl-3d0/devfreq/max_freq
    chmod 0664 /sys/class/kgsl/kgsl-3d0/devfreq/min_freq
    chmod 0664 /sys/class/kgsl/kgsl-3d0/devfreq/governor

    chmod 0664 /sys/class/power_supply/battery/constant_charge_current_max

    chmod 0644 /sys/class/thermal/thermal_message/sconfig

#############################LawRun-Starting####################################

## MOV on
mkdir -p /storage/emulated/0/LawRun-Kernel/
rm -r /storage/emulated/0/LawRun-Kernel/log.txt

# Profile Log
dt=`date '+%d/%m/%Y %H:%M:%S'`
echo "$dt LawRun profiles Started" >> /storage/emulated/0/LawRun-Kernel/log.txt

################################################################################

                          #####################
                          #                   #
                          #      Common       #
                          #                   #
                          #####################

###############################LawRun-Common####################################

# Setup final cpuset
echo "0-7" > /dev/cpuset/top-app/cpus
echo "0-3,6-7" > /dev/cpuset/foreground/boost/cpus
echo "0-3,6-7" > /dev/cpuset/foreground/cpus
echo "0-1" > /dev/cpuset/background/cpus
echo "0-3" > /dev/cpuset/system-background/cpus

# Runtime FS tuning: as we have init boottime setting and kernel patch setting
# default readahead to 2048KB. We should adjust the setting upon boot_complete
# for runtime performance
echo "512" > /sys/block/sda/queue/read_ahead_kb
echo "128" > /sys/block/sda/queue/nr_requests
echo "1" > /sys/block/sda/queue/iostats
echo "512" > /sys/block/sdb/queue/read_ahead_kb
echo "128" > /sys/block/sdb/queue/nr_requests
echo "1" > /sys/block/sdb/queue/iostats
echo "512" > /sys/block/sdc/queue/read_ahead_kb
echo "128" > /sys/block/sdc/queue/nr_requests
echo "1" > /sys/block/sdc/queue/iostats
echo "512" > /sys/block/sdd/queue/read_ahead_kb
echo "128" > /sys/block/sdd/queue/nr_requests
echo "1" > /sys/block/sdd/queue/iostats
echo "512" > /sys/block/sde/queue/read_ahead_kb
echo "128" > /sys/block/sde/queue/nr_requests
echo "1" > /sys/block/sde/queue/iostats
echo "512" > /sys/block/sdf/queue/read_ahead_kb
echo "128" > /sys/block/sdf/queue/nr_requests
echo "1" > /sys/block/sdf/queue/iostats

#Schedtune
echo "1" > /dev/stune/foreground/schedtune.prefer_idle
echo "1" > /dev/stune/top-app/schedtune.prefer_idle

# Set up block I/O cgroups
echo "0" > /dev/stune/blkio.group_idle
echo "1" > /dev/stune/foreground/blkio.group_idle
echo "0" > /dev/stune/background/blkio.group_idle
echo "2" > /dev/stune/top-app/blkio.group_idle
echo "2" > /dev/stune/rt/blkio.group_idle

echo "1000" > /dev/stune/blkio.weight
echo "1000" > /dev/stune/foreground/blkio.weight
echo "10" > /dev/stune/background/blkio.weight
echo "1000" > /dev/stune/top-app/blkio.weight
echo "1000" > /dev/stune/rt/blkio.weight

# Disable a few minor and overall pretty useless modules for slightly better battery life & system wide performance;
echo "Y" > /sys/module/bluetooth/parameters/disable_ertm
echo "Y" > /sys/module/bluetooth/parameters/disable_esco

# Enable display / screen panel power saving features;
echo "Y" > /sys/kernel/debug/dsi-panel-ebbg-fhd-ft8716-video_display/dsi-phy-0_allow_phy_power_off
echo "Y" > /sys/kernel/debug/dsi-panel-ebbg-fhd-ft8716-video_display/ulps_enable
echo "Y" > /sys/kernel/debug/dsi_panel_ebbg_fhd_ft8719_video_display/dsi-phy-0_allow_phy_power_off
echo "Y" > /sys/kernel/debug/dsi_panel_ebbg_fhd_ft8719_video_display/ulps_enable
echo "Y" > /sys/kernel/debug/dsi_panel_jdi_fhd_r63452_cmd_display/dsi-phy-0_allow_phy_power_off
echo "Y" > /sys/kernel/debug/dsi_panel_jdi_fhd_r63452_cmd_display/ulps_enable
echo "Y" > /sys/kernel/debug/dsi_panel_jdi_fhd_nt35596s_video_display/dsi-phy-0_allow_phy_power_off
echo "Y" > /sys/kernel/debug/dsi_panel_jdi_fhd_nt35596s_video_display/ulps_enable
echo "Y" > /sys/kernel/debug/dsi_tianma_fhd_nt36672a_video_display/dsi-phy-0_allow_phy_power_off
echo "Y" > /sys/kernel/debug/dsi_tianma_fhd_nt36672a_video_display/ulps_enable
echo "Y" > /sys/kernel/debug/dsi_ss_ea8074_notch_fhd_cmd_display/dsi-phy-0_allow_phy_power_off
echo "Y" > /sys/kernel/debug/dsi_ss_ea8074_notch_fhd_cmd_display/ulps_enable

## New

# Set the default IRQ affinity to the silver cluster.
write /proc/irq/default_smp_affinity f

# Disable sched stats for less overhead
write /proc/sys/kernel/sched_schedstats 0

# Thermals
write /sys/module/msm_thermal/core_control/enabled 0

# Wakelock
write /sys/class/misc/boeffla_wakelock_blocker/wakelock_blocker wlan_pno_wl;wlan_ipa;wcnss_filter_lock;[timerfd];hal_bluetooth_lock;IPA_WS;sensor_ind;wlan;netmgr_wl;qcom_rx_wakelock;wlan_wow_wl;wlan_extscan_wl;

# Gentle Fair Sleepers
write /sys/kernel/sched/gentle_fair_sleepers 0

# Sleep Disabled
write /sys/class/lcd/panel/power_reduce 1
write /sys/module/pm2/parameters/idle_sleep_mode Y
write /sys/power/autosleep mem
write /sys/power/mem_sleep deep

# Charge throttling
write /sys/module/smb_lib/parameters/skip_thermal 1

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
echo "2803200" > /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq

# BOost
echo "1056000" > /sys/module/cpu_input_boost/parameters/input_boost_freq_lp
echo "902400" > /sys/module/cpu_input_boost/parameters/input_boost_freq_hp
echo "100" > /sys/module/cpu_input_boost/parameters/input_boost_duration
echo "1500" > /sys/module/cpu_input_boost/parameters/dynamic_stune_boost_duration
echo "25" > /sys/module/cpu_input_boost/parameters/dynamic_stune_boost

# SILVER Cluster Limiter
echo "0" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/down_rate_limit_us
echo "1209000" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/hispeed_freq
echo "90" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/hispeed_load
echo "0" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/iowait_boost_enable
echo "1" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/pl
echo "0" > /sys/devices/system/cpu/cpu0/cpufreq/schedutil/up_rate_limit_us

# GOLD Cluster Limiter
echo "0" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/down_rate_limit_us
echo "1574000" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/hispeed_freq
echo "90" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/hispeed_load
echo "0" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/iowait_boost_enable
echo "1" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/pl
echo "0" > /sys/devices/system/cpu/cpu4/cpufreq/PIXEL_SMURFUTIL/up_rate_limit_us

################################################################################

                          #####################
                          #                   #
                          #        GPU        #
                          #                   #
                          #####################

################################################################################

# Low Freq
echo "180000000" > /sys/class/kgsl/kgsl-3d0/devfreq/min_freq 
echo "180000000" > /sys/class/kgsl/kgsl-3d0/min_gpuclk

# High Freq
echo "710000000" > /sys/class/kgsl/kgsl-3d0/devfreq/max_freq 
echo "710000000" > /sys/class/kgsl/kgsl-3d0/max_gpuclk

# Limiter
echo "0" > /sys/class/kgsl/kgsl-3d0/throttling
echo "8" > /sys/class/kgsl/kgsl-3d0/default_pwrlevel
echo "msm-adreno-tz" > /sys/class/kgsl/kgsl-3d0/devfreq/governor

################################################################################

                          #####################
                          #                   #
                          #    Schedulers     #
                          #                   #
                          #####################

################################################################################

# IO Scheduler
echo "maple" > /sys/block/sda/queue/scheduler
echo "maple" > /sys/block/sdb/queue/scheduler
echo "maple" > /sys/block/sdc/queue/scheduler
echo "maple" > /sys/block/sdd/queue/scheduler
echo "maple" > /sys/block/sde/queue/scheduler
echo "maple" > /sys/block/sdf/queue/scheduler

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
echo "Y" > /sys/module/workqueue/parameters/power_efficient

# Thermals
echo "-1" > /sys/class/thermal/thermal_message/sconfig

# Scale down in low write load
# That change tried to fix a problem for clock scaling during write requests.
# The default value for it is "0" (favor for downscale).
# For users who want performance over power they should set it to "1" (favor for upscale)
write /sys/class/mmc_host/mmc0/clk_scaling/scale_down_in_low_wr_load 0

# Panel Backlight
write /sys/class/leds/lcd-backlight/max_brightness 255

################################################################################

                          #####################
                          #                   #
                          #       Extra       #
                          #                   #
                          #####################

################################################################################

# LMK
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
echo 2147483648 > /sys/block/zram0/disksize
echo 2048M > /sys/block/zram0/mem_limit
echo 8 > /sys/block/zram0/max_comp_streams
mkswap /dev/block/zram0
swapon /dev/block/zram0 -p 32758
echo 65 > /proc/sys/vm/swappiness
echo 10 > /proc/sys/vm/dirty_background_ratio
echo 60 > /proc/sys/vm/vfs_cache_pressure
echo 3000 > /proc/sys/vm/dirty_writeback_centisecs

################################LawRun-END#######################################

sleep 20;

QSEECOMD=`pidof qseecomd`
THERMAL-ENGINE=`pidof thermal-engine`
TIME_DAEMON=`pidof time_daemon`
IMSQMIDAEMON=`pidof imsqmidaemon`
IMSDATADAEMON=`pidof imsdatadaemon`
DASHD=`pidof dashd`
CND=`pidof cnd`
DPMD=`pidof dpmd`
RMT_STORAGE=`pidof rmt_storage`
TFTP_SERVER=`pidof tftp_server`
NETMGRD=`pidof netmgrd`
IPACM=`pidof ipacm`
QTI=`pidof qti`
LOC_LAUNCHER=`pidof loc_launcher`
QSEEPROXYDAEMON=`pidof qseeproxydaemon`
IFAADAEMON=`pidof ifaadaemon`
LOGCAT=`pidof logcat`
LMKD=`pidof lmkd`
PERFD=`pidof perfd`
IOP=`pidof iop`
MSM_IRQBALANCE=`pidof msm_irqbalance`
SEEMP_HEALTHD=`pidof seemp_healthd`
ESEPMDAEMON=`pidof esepmdaemon`
WPA_SUPPLICANT=`pidof wpa_supplicant`
SEEMPD=`pidof seempd`
EMBRYO=`pidof embryo`
HEALTHD=`pidof healthd`
OEMLOGKIT=`pidof oemlogkit`
NETD=`pidof netd`

writepid_sbg $QSEECOMD;
writepid_sbg $THERMAL-ENGINE;
writepid_sbg $TIME_DAEMON;
writepid_sbg $IMSQMIDAEMON;
writepid_sbg $IMSDATADAEMON;
writepid_sbg $DASHD;
writepid_sbg $CND;
writepid_sbg $DPMD;
writepid_sbg $RMT_STORAGE;
writepid_sbg $TFTP_SERVER;
writepid_sbg $NETMGRD;
writepid_sbg $IPACM;
writepid_sbg $QTI;
writepid_sbg $LOC_LAUNCHER;
writepid_sbg $QSEEPROXYDAEMON;
writepid_sbg $IFAADAEMON;
writepid_sbg $LOGCAT;
writepid_sbg $LMKD;
writepid_sbg $PERFD;
writepid_sbg $IOP;
writepid_sbg $MSM_IRQBALANCE;
writepid_sbg $SEEMP_HEALTHD;
writepid_sbg $ESEPMDAEMON;
writepid_sbg $WPA_SUPPLICANT;
writepid_sbg $SEEMPD;
writepid_sbg $HEALTHD;
writepid_sbg $OEMLOGKIT;
writepid_sbg $NETD;

}&
