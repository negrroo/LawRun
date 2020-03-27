#!/system/bin/sh
# SPECTRUM KERNEL MANAGER
# Franco KERNEL MANAGER
# MORPHO KERNEL MANAGER
# Profile initialization script by nathanchance
# negrroo
# LawRun-Kernel Edit.

# If there is not a persist value, we need to set one

if [ ! -f /data/property/persist.spectrum.profile ]; then
    setprop persist.spectrum.profile 4
fi

if [ ! -f /data/property/persist.fku.profiles ]; then
		setprop persist.fku.profiles 3
fi

if [ ! -f /data/property/persist.morpho.profile ]; then
    setprop persist.morpho.profile 3
fi
