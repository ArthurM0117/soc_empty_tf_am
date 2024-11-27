################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include gecko_sdk_4.4.4/util/third_party/mbedtls/library/subdir.mk
-include gecko_sdk_4.4.4/util/silicon_labs/silabs_core/memory_manager/subdir.mk
-include gecko_sdk_4.4.4/protocol/bluetooth/src/subdir.mk
-include gecko_sdk_4.4.4/protocol/bluetooth/lib/EFR32XG12/GCC/subdir.mk
-include gecko_sdk_4.4.4/protocol/bluetooth/bgcommon/src/subdir.mk
-include gecko_sdk_4.4.4/platform/service/udelay/src/subdir.mk
-include gecko_sdk_4.4.4/platform/service/system/src/subdir.mk
-include gecko_sdk_4.4.4/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_4.4.4/platform/service/power_manager/src/subdir.mk
-include gecko_sdk_4.4.4/platform/service/mpu/src/subdir.mk
-include gecko_sdk_4.4.4/platform/service/iostream/src/subdir.mk
-include gecko_sdk_4.4.4/platform/service/device_init/src/subdir.mk
-include gecko_sdk_4.4.4/platform/security/sl_component/sl_psa_driver/src/subdir.mk
-include gecko_sdk_4.4.4/platform/security/sl_component/sl_protocol_crypto/src/subdir.mk
-include gecko_sdk_4.4.4/platform/security/sl_component/sl_mbedtls_support/src/subdir.mk
-include gecko_sdk_4.4.4/platform/radio/rail_lib/plugin/rail_util_pti/subdir.mk
-include gecko_sdk_4.4.4/platform/radio/rail_lib/plugin/rail_util_power_manager_init/subdir.mk
-include gecko_sdk_4.4.4/platform/radio/rail_lib/plugin/pa-conversions/subdir.mk
-include gecko_sdk_4.4.4/platform/emlib/src/subdir.mk
-include gecko_sdk_4.4.4/platform/emdrv/dmadrv/src/subdir.mk
-include gecko_sdk_4.4.4/platform/driver/i2cspm/src/subdir.mk
-include gecko_sdk_4.4.4/platform/driver/debug/src/subdir.mk
-include gecko_sdk_4.4.4/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_4.4.4/platform/common/src/subdir.mk
-include gecko_sdk_4.4.4/platform/bootloader/app_properties/subdir.mk
-include gecko_sdk_4.4.4/platform/bootloader/api/subdir.mk
-include gecko_sdk_4.4.4/platform/Device/SiliconLabs/EFR32MG12P/Source/subdir.mk
-include gecko_sdk_4.4.4/hardware/driver/si70xx/src/subdir.mk
-include gecko_sdk_4.4.4/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/subdir.mk
-include gecko_sdk_4.4.4/hardware/driver/configuration_over_swo/src/subdir.mk
-include gecko_sdk_4.4.4/hardware/board/src/subdir.mk
-include gecko_sdk_4.4.4/app/common/util/app_timer/subdir.mk
-include gecko_sdk_4.4.4/app/common/util/app_log/subdir.mk
-include gecko_sdk_4.4.4/app/bluetooth/common/sensor_select/subdir.mk
-include gecko_sdk_4.4.4/app/bluetooth/common/sensor_rht/subdir.mk
-include gecko_sdk_4.4.4/app/bluetooth/common/in_place_ota_dfu/subdir.mk
-include autogen/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := soc_empty_tf_am
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: main-build

# Main-build Target
main-build: soc_empty_tf_am.axf

# Tool invocations
soc_empty_tf_am.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\arthu\SimplicityStudio\v5_workspace\soc_empty_tf_am\autogen\linkerfile.ld" -Wl,--no-warn-rwx-segments -Xlinker --gc-sections -Xlinker -Map="soc_empty_tf_am.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o soc_empty_tf_am.axf -Wl,--start-group "./app.o" "./main.o" "./sl_gatt_service_device_information.o" "./temperature.o" "./autogen/gatt_db.o" "./autogen/sl_bluetooth.o" "./autogen/sl_board_default_init.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_event_handler.o" "./autogen/sl_i2cspm_init.o" "./autogen/sl_iostream_handles.o" "./autogen/sl_iostream_init_usart_instances.o" "./autogen/sl_power_manager_handler.o" "./gecko_sdk_4.4.4/app/bluetooth/common/in_place_ota_dfu/sl_bt_in_place_ota_dfu.o" "./gecko_sdk_4.4.4/app/bluetooth/common/sensor_rht/sl_sensor_rht.o" "./gecko_sdk_4.4.4/app/bluetooth/common/sensor_select/sl_sensor_select.o" "./gecko_sdk_4.4.4/app/common/util/app_log/app_log.o" "./gecko_sdk_4.4.4/app/common/util/app_timer/app_timer.o" "./gecko_sdk_4.4.4/hardware/board/src/brd4166a_support.o" "./gecko_sdk_4.4.4/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_4.4.4/hardware/board/src/sl_board_init.o" "./gecko_sdk_4.4.4/hardware/driver/configuration_over_swo/src/sl_cos.o" "./gecko_sdk_4.4.4/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/sl_mx25_flash_shutdown.o" "./gecko_sdk_4.4.4/hardware/driver/si70xx/src/sl_si70xx.o" "./gecko_sdk_4.4.4/platform/Device/SiliconLabs/EFR32MG12P/Source/startup_efr32mg12p.o" "./gecko_sdk_4.4.4/platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.o" "./gecko_sdk_4.4.4/platform/bootloader/api/btl_interface.o" "./gecko_sdk_4.4.4/platform/bootloader/api/btl_interface_storage.o" "./gecko_sdk_4.4.4/platform/bootloader/app_properties/app_properties.o" "./gecko_sdk_4.4.4/platform/common/src/sl_assert.o" "./gecko_sdk_4.4.4/platform/common/src/sl_slist.o" "./gecko_sdk_4.4.4/platform/common/src/sl_string.o" "./gecko_sdk_4.4.4/platform/common/src/sl_syscalls.o" "./gecko_sdk_4.4.4/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_4.4.4/platform/driver/debug/src/sl_debug_swo.o" "./gecko_sdk_4.4.4/platform/driver/i2cspm/src/sl_i2cspm.o" "./gecko_sdk_4.4.4/platform/emdrv/dmadrv/src/dmadrv.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_acmp.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_adc.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_cmu.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_core.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_cryotimer.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_crypto.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_csen.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_dbg.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_emu.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_gpcrc.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_gpio.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_i2c.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_idac.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_ldma.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_lesense.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_letimer.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_leuart.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_msc.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_opamp.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_pcnt.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_prs.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_rmu.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_rtcc.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_system.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_timer.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_usart.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_vdac.o" "./gecko_sdk_4.4.4/platform/emlib/src/em_wdog.o" "./gecko_sdk_4.4.4/platform/radio/rail_lib/plugin/pa-conversions/pa_conversions_efr32.o" "./gecko_sdk_4.4.4/platform/radio/rail_lib/plugin/pa-conversions/pa_curves_efr32.o" "./gecko_sdk_4.4.4/platform/radio/rail_lib/plugin/rail_util_power_manager_init/sl_rail_util_power_manager_init.o" "./gecko_sdk_4.4.4/platform/radio/rail_lib/plugin/rail_util_pti/sl_rail_util_pti.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_mbedtls_support/src/crypto_aes.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_mbedtls_support/src/crypto_ecp.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_mbedtls_support/src/mbedtls_cmac.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_mbedtls_support/src/mbedtls_sha.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_mbedtls_support/src/sl_entropy_hardware.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_mbedtls_support/src/sl_mbedtls.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_protocol_crypto/src/sli_protocol_crypto_crypto.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_psa_driver/src/crypto_management.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_psa_driver/src/sli_crypto_driver_trng.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_aead.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_cipher.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_hash.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_mac.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_psa_driver/src/sli_psa_driver_common.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_psa_driver/src/sli_psa_driver_init.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_psa_driver/src/sli_psa_trng.o" "./gecko_sdk_4.4.4/platform/security/sl_component/sl_psa_driver/src/sli_se_version_dependencies.o" "./gecko_sdk_4.4.4/platform/service/device_init/src/sl_device_init_dcdc_s1.o" "./gecko_sdk_4.4.4/platform/service/device_init/src/sl_device_init_emu_s1.o" "./gecko_sdk_4.4.4/platform/service/device_init/src/sl_device_init_hfxo_s1.o" "./gecko_sdk_4.4.4/platform/service/device_init/src/sl_device_init_lfxo_s1.o" "./gecko_sdk_4.4.4/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_4.4.4/platform/service/iostream/src/sl_iostream.o" "./gecko_sdk_4.4.4/platform/service/iostream/src/sl_iostream_retarget_stdio.o" "./gecko_sdk_4.4.4/platform/service/iostream/src/sl_iostream_stdlib_config.o" "./gecko_sdk_4.4.4/platform/service/iostream/src/sl_iostream_uart.o" "./gecko_sdk_4.4.4/platform/service/iostream/src/sl_iostream_usart.o" "./gecko_sdk_4.4.4/platform/service/mpu/src/sl_mpu.o" "./gecko_sdk_4.4.4/platform/service/power_manager/src/sl_power_manager.o" "./gecko_sdk_4.4.4/platform/service/power_manager/src/sl_power_manager_debug.o" "./gecko_sdk_4.4.4/platform/service/power_manager/src/sl_power_manager_hal_s0_s1.o" "./gecko_sdk_4.4.4/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_4.4.4/platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "./gecko_sdk_4.4.4/platform/service/sleeptimer/src/sl_sleeptimer_hal_timer.o" "./gecko_sdk_4.4.4/platform/service/system/src/sl_system_init.o" "./gecko_sdk_4.4.4/platform/service/system/src/sl_system_process_action.o" "./gecko_sdk_4.4.4/platform/service/udelay/src/sl_udelay.o" "./gecko_sdk_4.4.4/platform/service/udelay/src/sl_udelay_armv6m_gcc.o" "./gecko_sdk_4.4.4/protocol/bluetooth/bgcommon/src/sl_bt_mbedtls_context.o" "./gecko_sdk_4.4.4/protocol/bluetooth/src/sl_apploader_util_s1.o" "./gecko_sdk_4.4.4/protocol/bluetooth/src/sl_bt_stack_init.o" "./gecko_sdk_4.4.4/protocol/bluetooth/src/sli_bt_accept_list_config.o" "./gecko_sdk_4.4.4/protocol/bluetooth/src/sli_bt_advertiser_config.o" "./gecko_sdk_4.4.4/protocol/bluetooth/src/sli_bt_connection_config.o" "./gecko_sdk_4.4.4/protocol/bluetooth/src/sli_bt_dynamic_gattdb_config.o" "./gecko_sdk_4.4.4/protocol/bluetooth/src/sli_bt_l2cap_config.o" "./gecko_sdk_4.4.4/protocol/bluetooth/src/sli_bt_pawr_advertiser_config.o" "./gecko_sdk_4.4.4/protocol/bluetooth/src/sli_bt_periodic_adv_config.o" "./gecko_sdk_4.4.4/protocol/bluetooth/src/sli_bt_periodic_advertiser_config.o" "./gecko_sdk_4.4.4/protocol/bluetooth/src/sli_bt_sync_config.o" "./gecko_sdk_4.4.4/util/silicon_labs/silabs_core/memory_manager/sl_malloc.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/aes.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/bignum.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/bignum_core.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/bignum_mod.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/bignum_mod_raw.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/cipher.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/cipher_wrap.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/cmac.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/constant_time.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/ctr_drbg.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/ecdh.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/ecp.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/ecp_curves.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/ecp_curves_new.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/entropy.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/entropy_poll.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/hmac_drbg.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/md.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/platform.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/platform_util.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/psa_crypto_client.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/psa_util.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/sha256.o" "./gecko_sdk_4.4.4/util/third_party/mbedtls/library/threading.o" "C:/Users/arthu/SimplicityStudio/SDKs/gecko_sdk//protocol/bluetooth/lib/EFR32XG12/GCC/binapploader.o" "C:/Users/arthu/SimplicityStudio/SDKs/gecko_sdk//protocol/bluetooth/bgcommon/lib/libbgcommon_efr32xg12_gcc_release.a" "C:/Users/arthu/SimplicityStudio/SDKs/gecko_sdk//protocol/bluetooth/bgstack/ll/lib/libbluetooth_controller_efr32xg12_gcc_release.a" "C:/Users/arthu/SimplicityStudio/SDKs/gecko_sdk//protocol/bluetooth/lib/libbluetooth_host_efr32xg12_gcc_release.a" "C:/Users/arthu/SimplicityStudio/SDKs/gecko_sdk//protocol/bluetooth/lib/EFR32XG12/GCC/libpsstore.a" "C:/Users/arthu/SimplicityStudio/SDKs/gecko_sdk//platform/radio/rail_lib/autogen/librail_release/librail_efr32xg12_gcc_release.a" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building out file: soc_empty_tf_am.out'
	arm-none-eabi-objcopy "soc_empty_tf_am.axf" "soc_empty_tf_am.out"
	@echo ' '

	@echo 'Building hex file: soc_empty_tf_am.hex'
	arm-none-eabi-objcopy -O ihex "soc_empty_tf_am.axf" "soc_empty_tf_am.hex"
	@echo ' '

	@echo 'Building bin file: soc_empty_tf_am.bin'
	arm-none-eabi-objcopy -O binary "soc_empty_tf_am.axf" "soc_empty_tf_am.bin"
	@echo ' '

	@echo 'Building s37 file: soc_empty_tf_am.s37'
	arm-none-eabi-objcopy -O srec "soc_empty_tf_am.axf" "soc_empty_tf_am.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "soc_empty_tf_am.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) soc_empty_tf_am.axf
	-@echo ' '

.PHONY: all clean dependents main-build

-include ../makefile.targets
