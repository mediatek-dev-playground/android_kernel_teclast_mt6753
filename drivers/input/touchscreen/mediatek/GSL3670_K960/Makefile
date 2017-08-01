include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GSL3670_K960/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GSL3670_K960/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GSL3670_K960/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

#target
obj-y   += mtk_gsl3670.o
obj-y	+= gsl_point_id.o
