################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Crystalfontz128x128_ST7735.obj: ../Crystalfontz128x128_ST7735.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O1 --opt_for_speed=2 --include_path="C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates" --include_path="C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates" --include_path="C:/TI/tirtos_tivac_2_16_00_08/products/TivaWare_C_Series-2.1.1.71b" --include_path="C:/TI/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages/ti/sysbios/posix" --include_path="D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=ccs --define=TIVAWARE -g --gcc --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="Crystalfontz128x128_ST7735.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.obj: ../HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O1 --opt_for_speed=2 --include_path="C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates" --include_path="C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates" --include_path="C:/TI/tirtos_tivac_2_16_00_08/products/TivaWare_C_Series-2.1.1.71b" --include_path="C:/TI/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages/ti/sysbios/posix" --include_path="D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=ccs --define=TIVAWARE -g --gcc --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="HAL_EK_TM4C1294XL_Crystalfontz128x128_ST7735.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

kiss_fft.obj: ../kiss_fft.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O1 --opt_for_speed=2 --include_path="C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates" --include_path="C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates" --include_path="C:/TI/tirtos_tivac_2_16_00_08/products/TivaWare_C_Series-2.1.1.71b" --include_path="C:/TI/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages/ti/sysbios/posix" --include_path="D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=ccs --define=TIVAWARE -g --gcc --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="kiss_fft.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

main.obj: ../main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O1 --opt_for_speed=2 --include_path="C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates" --include_path="C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates" --include_path="C:/TI/tirtos_tivac_2_16_00_08/products/TivaWare_C_Series-2.1.1.71b" --include_path="C:/TI/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages/ti/sysbios/posix" --include_path="D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=ccs --define=TIVAWARE -g --gcc --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="main.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1006130751:
	@$(MAKE) --no-print-directory -Onone -f subdir_rules.mk build-1006130751-inproc

build-1006130751-inproc: ../rtos.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"C:/ti/xdctools_3_32_00_06_core/xs" --xdcpath="C:/TI/tirtos_tivac_2_16_00_08/packages;C:/TI/tirtos_tivac_2_16_00_08/products/tidrivers_tivac_2_16_00_08/packages;C:/TI/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages;C:/TI/tirtos_tivac_2_16_00_08/products/ndk_2_25_00_09/packages;C:/TI/tirtos_tivac_2_16_00_08/products/uia_2_00_05_50/packages;C:/TI/tirtos_tivac_2_16_00_08/products/ns_1_11_00_10/packages;D:/Programs/CCSv8/ccsv8/ccs_base;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M4F -p ti.platforms.tiva:TM4C1294NCPDT -r release -c "D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS" --compileOptions "-mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O1 --opt_for_speed=2 --include_path=\"C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates\" --include_path=\"C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates\" --include_path=\"C:/TI/tirtos_tivac_2_16_00_08/products/TivaWare_C_Series-2.1.1.71b\" --include_path=\"C:/TI/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages/ti/sysbios/posix\" --include_path=\"D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include\" --define=ccs=\"ccs\" --define=PART_TM4C1294NCPDT --define=ccs --define=TIVAWARE -g --gcc --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi  " "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-1006130751 ../rtos.cfg
configPkg/compiler.opt: build-1006130751
configPkg/: build-1006130751

sysctl_pll.obj: ../sysctl_pll.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O1 --opt_for_speed=2 --include_path="C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates" --include_path="C:/Users/doate/Desktop/ECE-3849/ECE-3849-Labs/ece3849d19_lab2_mrhagan_doates" --include_path="C:/TI/tirtos_tivac_2_16_00_08/products/TivaWare_C_Series-2.1.1.71b" --include_path="C:/TI/tirtos_tivac_2_16_00_08/products/bios_6_45_01_29/packages/ti/sysbios/posix" --include_path="D:/Programs/CCSv8/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=ccs --define=TIVAWARE -g --gcc --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="sysctl_pll.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


