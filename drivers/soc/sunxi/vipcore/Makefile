# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_ALLWINNER_VIPCORE) += vipcore.o

ccflags-y += -Wno-error -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable
ccflags-y += -I$(srctree)/drivers/soc/sunxi/vipcore \
             -I$(srctree)/drivers/soc/sunxi/vipcore/inc \
             -I$(srctree)/drivers/soc/sunxi/vipcore/memory \
             -I$(srctree)/drivers/soc/sunxi/vipcore/os/linux \
             -I$(srctree)/drivers/soc/sunxi/vipcore/os/linux/allocator \
             -I$(srctree)/drivers/soc/sunxi/vipcore/os/linux/platform/allwinner \
             -I$(srctree)/drivers/soc/sunxi/vipcore/task

ccflags-y += -DLINUX -DDRIVER -DCONFIG_ARCH_SUN60IW2=1
ccflags-y += -DUSE_LINUX_PLATFORM_DEVICE
ccflags-y += -DvpmdAUTO_CORRECT_CONFLICTS=1
ccflags-y += -DvpmdENABLE_USE_AGENT_TRIGGER=1

vipcore-objs := vip_drv_interface.o \
                vip_drv_hardware.o \
                vip_drv_device.o \
                vip_drv_context.o \
                vip_drv_debug.o \
                vip_drv_util.o \
                vip_drv_pm.o \
                memory/vip_drv_mem_heap.o \
                memory/vip_drv_mmu.o \
                memory/vip_drv_video_memory.o \
                memory/vip_drv_mem_allocator.o \
                task/vip_drv_task_descriptor.o \
                task/vip_drv_task_schedule.o \
                task/vip_drv_task_parallel_async.o \
                task/vip_drv_task_serial_sync.o \
                task/vip_drv_task_parallel_sync.o \
                task/vip_drv_task_single.o \
                task/vip_drv_task_common.o \
                task/vip_drv_task_debug.o \
                os/linux/vip_drv_device_driver.o \
                os/linux/vip_drv_os.o \
                os/linux/vip_drv_os_debug.o \
                os/linux/allocator/vip_drv_mem_allocator_common.o \
                os/linux/allocator/vip_drv_mem_allocator_heap_reserved.o \
                os/linux/allocator/vip_drv_mem_allocator_heap_exclusive.o \
                os/linux/allocator/vip_drv_mem_allocator_continue.o \
                os/linux/allocator/vip_drv_mem_allocator_segment_continue.o \
                os/linux/allocator/vip_drv_mem_allocator_gfp.o \
                os/linux/allocator/vip_drv_mem_allocator_cma.o \
                os/linux/allocator/vip_drv_mem_allocator_ion.o \
                os/linux/allocator/vip_drv_mem_allocator_wrap_user_physical.o \
                os/linux/allocator/vip_drv_mem_allocator_wrap_user_memory.o \
                os/linux/allocator/vip_drv_mem_allocator_wrap_dma_buf.o \
                os/linux/platform/allwinner/vip_drv_device_platform.o
