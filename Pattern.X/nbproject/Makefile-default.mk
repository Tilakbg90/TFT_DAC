#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Pattern.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Pattern.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED="../../../Board Support Package/TouchScreen.c" "../../../Board Support Package/TouchScreenAR1020.c" "../../../Board Support Package/TouchScreenResistive.c" "../../../Board Support Package/Beep.c" "../../../Board Support Package/SST25VF016.c" "../../../Board Support Package/cpld.c" "../../../Board Support Package/drv_spi.c" "../../../Board Support Package/SST39LF400.c" "../../../Board Support Package/SST39VF040.c" "../../../Board Support Package/MCHP25LC256.c" SST26VF016B.c ../../../Microchip/Common/TimeDelay.c ../../../Microchip/Graphics/Drivers/gfxepmp.c ../../../Microchip/Graphics/Drivers/mchpGfxDrv.c ../../../Microchip/Graphics/Drivers/mchpGfxLCC.c ../../../Microchip/Graphics/Drivers/TCON_Custom.c ../../../Microchip/Graphics/Drivers/TCON_HX8238.c ../../../Microchip/Graphics/Drivers/TCON_HX8257.c ../../../Microchip/Graphics/Drivers/TCON_SSD1289.c ../../../Microchip/Graphics/Drivers/drvTFT001.c ../../../Microchip/Graphics/Drivers/drvTFT002.c ../../../Microchip/Graphics/Drivers/HIT1270.c ../../../Microchip/Graphics/Drivers/HX8347.c ../../../Microchip/Graphics/Drivers/S1D13517.c ../../../Microchip/Graphics/Drivers/SH1101A_SSD1303.c ../../../Microchip/Graphics/Drivers/SSD1339.c ../../../Microchip/Graphics/Drivers/SSD1926.c ../../../Microchip/Graphics/Drivers/ST7529.c ../../../Microchip/Graphics/Drivers/UC1610.c ../../../Microchip/Graphics/GOL.c ../../../Microchip/Graphics/GOLFontDefault.c ../../../Microchip/Graphics/GOLSchemeDefault.c ../../../Microchip/Graphics/Palette.c ../../../Microchip/Graphics/Primitive.c ../../../Microchip/Graphics/Button.c ../../../Microchip/Graphics/CheckBox.c ../../../Microchip/Graphics/Meter.c ../../../Microchip/Graphics/DigitalMeter.c ../../../Microchip/Graphics/Chart.c ../../../Microchip/Graphics/RadioButton.c ../../../Microchip/Graphics/ListBox.c ../../../Microchip/Graphics/GroupBox.c ../../../Microchip/Graphics/EditBox.c ../../../Microchip/Graphics/Window.c ../../../Microchip/Graphics/Slider.c ../../../Microchip/Graphics/StaticText.c ../../../Microchip/Graphics/RoundDial.c ../../../Microchip/Graphics/Picture.c ../../../Microchip/Graphics/ProgressBar.c ../../../Microchip/Graphics/TextEntry.c Main.c DRV_SPI_SMCPU.c CRC16.c Pattern/GDD_Screens.c Pattern/GDD_Resource.c Pattern/GDD_X_Event_Handler.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1687970324/TouchScreen.o ${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o ${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o ${OBJECTDIR}/_ext/1687970324/Beep.o ${OBJECTDIR}/_ext/1687970324/SST25VF016.o ${OBJECTDIR}/_ext/1687970324/cpld.o ${OBJECTDIR}/_ext/1687970324/drv_spi.o ${OBJECTDIR}/_ext/1687970324/SST39LF400.o ${OBJECTDIR}/_ext/1687970324/SST39VF040.o ${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o ${OBJECTDIR}/SST26VF016B.o ${OBJECTDIR}/_ext/1860783399/TimeDelay.o ${OBJECTDIR}/_ext/1919882987/gfxepmp.o ${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o ${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o ${OBJECTDIR}/_ext/1919882987/TCON_Custom.o ${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o ${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o ${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o ${OBJECTDIR}/_ext/1919882987/drvTFT001.o ${OBJECTDIR}/_ext/1919882987/drvTFT002.o ${OBJECTDIR}/_ext/1919882987/HIT1270.o ${OBJECTDIR}/_ext/1919882987/HX8347.o ${OBJECTDIR}/_ext/1919882987/S1D13517.o ${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o ${OBJECTDIR}/_ext/1919882987/SSD1339.o ${OBJECTDIR}/_ext/1919882987/SSD1926.o ${OBJECTDIR}/_ext/1919882987/ST7529.o ${OBJECTDIR}/_ext/1919882987/UC1610.o ${OBJECTDIR}/_ext/803622919/GOL.o ${OBJECTDIR}/_ext/803622919/GOLFontDefault.o ${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o ${OBJECTDIR}/_ext/803622919/Palette.o ${OBJECTDIR}/_ext/803622919/Primitive.o ${OBJECTDIR}/_ext/803622919/Button.o ${OBJECTDIR}/_ext/803622919/CheckBox.o ${OBJECTDIR}/_ext/803622919/Meter.o ${OBJECTDIR}/_ext/803622919/DigitalMeter.o ${OBJECTDIR}/_ext/803622919/Chart.o ${OBJECTDIR}/_ext/803622919/RadioButton.o ${OBJECTDIR}/_ext/803622919/ListBox.o ${OBJECTDIR}/_ext/803622919/GroupBox.o ${OBJECTDIR}/_ext/803622919/EditBox.o ${OBJECTDIR}/_ext/803622919/Window.o ${OBJECTDIR}/_ext/803622919/Slider.o ${OBJECTDIR}/_ext/803622919/StaticText.o ${OBJECTDIR}/_ext/803622919/RoundDial.o ${OBJECTDIR}/_ext/803622919/Picture.o ${OBJECTDIR}/_ext/803622919/ProgressBar.o ${OBJECTDIR}/_ext/803622919/TextEntry.o ${OBJECTDIR}/Main.o ${OBJECTDIR}/DRV_SPI_SMCPU.o ${OBJECTDIR}/CRC16.o ${OBJECTDIR}/Pattern/GDD_Screens.o ${OBJECTDIR}/Pattern/GDD_Resource.o ${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1687970324/TouchScreen.o.d ${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o.d ${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o.d ${OBJECTDIR}/_ext/1687970324/Beep.o.d ${OBJECTDIR}/_ext/1687970324/SST25VF016.o.d ${OBJECTDIR}/_ext/1687970324/cpld.o.d ${OBJECTDIR}/_ext/1687970324/drv_spi.o.d ${OBJECTDIR}/_ext/1687970324/SST39LF400.o.d ${OBJECTDIR}/_ext/1687970324/SST39VF040.o.d ${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o.d ${OBJECTDIR}/SST26VF016B.o.d ${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d ${OBJECTDIR}/_ext/1919882987/gfxepmp.o.d ${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o.d ${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o.d ${OBJECTDIR}/_ext/1919882987/TCON_Custom.o.d ${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o.d ${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o.d ${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o.d ${OBJECTDIR}/_ext/1919882987/drvTFT001.o.d ${OBJECTDIR}/_ext/1919882987/drvTFT002.o.d ${OBJECTDIR}/_ext/1919882987/HIT1270.o.d ${OBJECTDIR}/_ext/1919882987/HX8347.o.d ${OBJECTDIR}/_ext/1919882987/S1D13517.o.d ${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o.d ${OBJECTDIR}/_ext/1919882987/SSD1339.o.d ${OBJECTDIR}/_ext/1919882987/SSD1926.o.d ${OBJECTDIR}/_ext/1919882987/ST7529.o.d ${OBJECTDIR}/_ext/1919882987/UC1610.o.d ${OBJECTDIR}/_ext/803622919/GOL.o.d ${OBJECTDIR}/_ext/803622919/GOLFontDefault.o.d ${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o.d ${OBJECTDIR}/_ext/803622919/Palette.o.d ${OBJECTDIR}/_ext/803622919/Primitive.o.d ${OBJECTDIR}/_ext/803622919/Button.o.d ${OBJECTDIR}/_ext/803622919/CheckBox.o.d ${OBJECTDIR}/_ext/803622919/Meter.o.d ${OBJECTDIR}/_ext/803622919/DigitalMeter.o.d ${OBJECTDIR}/_ext/803622919/Chart.o.d ${OBJECTDIR}/_ext/803622919/RadioButton.o.d ${OBJECTDIR}/_ext/803622919/ListBox.o.d ${OBJECTDIR}/_ext/803622919/GroupBox.o.d ${OBJECTDIR}/_ext/803622919/EditBox.o.d ${OBJECTDIR}/_ext/803622919/Window.o.d ${OBJECTDIR}/_ext/803622919/Slider.o.d ${OBJECTDIR}/_ext/803622919/StaticText.o.d ${OBJECTDIR}/_ext/803622919/RoundDial.o.d ${OBJECTDIR}/_ext/803622919/Picture.o.d ${OBJECTDIR}/_ext/803622919/ProgressBar.o.d ${OBJECTDIR}/_ext/803622919/TextEntry.o.d ${OBJECTDIR}/Main.o.d ${OBJECTDIR}/DRV_SPI_SMCPU.o.d ${OBJECTDIR}/CRC16.o.d ${OBJECTDIR}/Pattern/GDD_Screens.o.d ${OBJECTDIR}/Pattern/GDD_Resource.o.d ${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1687970324/TouchScreen.o ${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o ${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o ${OBJECTDIR}/_ext/1687970324/Beep.o ${OBJECTDIR}/_ext/1687970324/SST25VF016.o ${OBJECTDIR}/_ext/1687970324/cpld.o ${OBJECTDIR}/_ext/1687970324/drv_spi.o ${OBJECTDIR}/_ext/1687970324/SST39LF400.o ${OBJECTDIR}/_ext/1687970324/SST39VF040.o ${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o ${OBJECTDIR}/SST26VF016B.o ${OBJECTDIR}/_ext/1860783399/TimeDelay.o ${OBJECTDIR}/_ext/1919882987/gfxepmp.o ${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o ${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o ${OBJECTDIR}/_ext/1919882987/TCON_Custom.o ${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o ${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o ${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o ${OBJECTDIR}/_ext/1919882987/drvTFT001.o ${OBJECTDIR}/_ext/1919882987/drvTFT002.o ${OBJECTDIR}/_ext/1919882987/HIT1270.o ${OBJECTDIR}/_ext/1919882987/HX8347.o ${OBJECTDIR}/_ext/1919882987/S1D13517.o ${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o ${OBJECTDIR}/_ext/1919882987/SSD1339.o ${OBJECTDIR}/_ext/1919882987/SSD1926.o ${OBJECTDIR}/_ext/1919882987/ST7529.o ${OBJECTDIR}/_ext/1919882987/UC1610.o ${OBJECTDIR}/_ext/803622919/GOL.o ${OBJECTDIR}/_ext/803622919/GOLFontDefault.o ${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o ${OBJECTDIR}/_ext/803622919/Palette.o ${OBJECTDIR}/_ext/803622919/Primitive.o ${OBJECTDIR}/_ext/803622919/Button.o ${OBJECTDIR}/_ext/803622919/CheckBox.o ${OBJECTDIR}/_ext/803622919/Meter.o ${OBJECTDIR}/_ext/803622919/DigitalMeter.o ${OBJECTDIR}/_ext/803622919/Chart.o ${OBJECTDIR}/_ext/803622919/RadioButton.o ${OBJECTDIR}/_ext/803622919/ListBox.o ${OBJECTDIR}/_ext/803622919/GroupBox.o ${OBJECTDIR}/_ext/803622919/EditBox.o ${OBJECTDIR}/_ext/803622919/Window.o ${OBJECTDIR}/_ext/803622919/Slider.o ${OBJECTDIR}/_ext/803622919/StaticText.o ${OBJECTDIR}/_ext/803622919/RoundDial.o ${OBJECTDIR}/_ext/803622919/Picture.o ${OBJECTDIR}/_ext/803622919/ProgressBar.o ${OBJECTDIR}/_ext/803622919/TextEntry.o ${OBJECTDIR}/Main.o ${OBJECTDIR}/DRV_SPI_SMCPU.o ${OBJECTDIR}/CRC16.o ${OBJECTDIR}/Pattern/GDD_Screens.o ${OBJECTDIR}/Pattern/GDD_Resource.o ${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o

# Source Files
SOURCEFILES=../../../Board Support Package/TouchScreen.c ../../../Board Support Package/TouchScreenAR1020.c ../../../Board Support Package/TouchScreenResistive.c ../../../Board Support Package/Beep.c ../../../Board Support Package/SST25VF016.c ../../../Board Support Package/cpld.c ../../../Board Support Package/drv_spi.c ../../../Board Support Package/SST39LF400.c ../../../Board Support Package/SST39VF040.c ../../../Board Support Package/MCHP25LC256.c SST26VF016B.c ../../../Microchip/Common/TimeDelay.c ../../../Microchip/Graphics/Drivers/gfxepmp.c ../../../Microchip/Graphics/Drivers/mchpGfxDrv.c ../../../Microchip/Graphics/Drivers/mchpGfxLCC.c ../../../Microchip/Graphics/Drivers/TCON_Custom.c ../../../Microchip/Graphics/Drivers/TCON_HX8238.c ../../../Microchip/Graphics/Drivers/TCON_HX8257.c ../../../Microchip/Graphics/Drivers/TCON_SSD1289.c ../../../Microchip/Graphics/Drivers/drvTFT001.c ../../../Microchip/Graphics/Drivers/drvTFT002.c ../../../Microchip/Graphics/Drivers/HIT1270.c ../../../Microchip/Graphics/Drivers/HX8347.c ../../../Microchip/Graphics/Drivers/S1D13517.c ../../../Microchip/Graphics/Drivers/SH1101A_SSD1303.c ../../../Microchip/Graphics/Drivers/SSD1339.c ../../../Microchip/Graphics/Drivers/SSD1926.c ../../../Microchip/Graphics/Drivers/ST7529.c ../../../Microchip/Graphics/Drivers/UC1610.c ../../../Microchip/Graphics/GOL.c ../../../Microchip/Graphics/GOLFontDefault.c ../../../Microchip/Graphics/GOLSchemeDefault.c ../../../Microchip/Graphics/Palette.c ../../../Microchip/Graphics/Primitive.c ../../../Microchip/Graphics/Button.c ../../../Microchip/Graphics/CheckBox.c ../../../Microchip/Graphics/Meter.c ../../../Microchip/Graphics/DigitalMeter.c ../../../Microchip/Graphics/Chart.c ../../../Microchip/Graphics/RadioButton.c ../../../Microchip/Graphics/ListBox.c ../../../Microchip/Graphics/GroupBox.c ../../../Microchip/Graphics/EditBox.c ../../../Microchip/Graphics/Window.c ../../../Microchip/Graphics/Slider.c ../../../Microchip/Graphics/StaticText.c ../../../Microchip/Graphics/RoundDial.c ../../../Microchip/Graphics/Picture.c ../../../Microchip/Graphics/ProgressBar.c ../../../Microchip/Graphics/TextEntry.c Main.c DRV_SPI_SMCPU.c CRC16.c Pattern/GDD_Screens.c Pattern/GDD_Resource.c Pattern/GDD_X_Event_Handler.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Pattern.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24FJ256DA210
MP_LINKER_FILE_OPTION=,--script=p24FJ256DA210.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1687970324/TouchScreen.o: ../../../Board\ Support\ Package/TouchScreen.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreen.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreen.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/TouchScreen.c"  -o ${OBJECTDIR}/_ext/1687970324/TouchScreen.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/TouchScreen.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/TouchScreen.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o: ../../../Board\ Support\ Package/TouchScreenAR1020.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/TouchScreenAR1020.c"  -o ${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o: ../../../Board\ Support\ Package/TouchScreenResistive.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/TouchScreenResistive.c"  -o ${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/Beep.o: ../../../Board\ Support\ Package/Beep.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/Beep.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/Beep.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/Beep.c"  -o ${OBJECTDIR}/_ext/1687970324/Beep.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/Beep.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/Beep.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/SST25VF016.o: ../../../Board\ Support\ Package/SST25VF016.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST25VF016.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST25VF016.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/SST25VF016.c"  -o ${OBJECTDIR}/_ext/1687970324/SST25VF016.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/SST25VF016.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/SST25VF016.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/cpld.o: ../../../Board\ Support\ Package/cpld.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/cpld.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/cpld.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/cpld.c"  -o ${OBJECTDIR}/_ext/1687970324/cpld.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/cpld.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/cpld.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/drv_spi.o: ../../../Board\ Support\ Package/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/drv_spi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/drv_spi.c"  -o ${OBJECTDIR}/_ext/1687970324/drv_spi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/drv_spi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/drv_spi.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/SST39LF400.o: ../../../Board\ Support\ Package/SST39LF400.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST39LF400.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST39LF400.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/SST39LF400.c"  -o ${OBJECTDIR}/_ext/1687970324/SST39LF400.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/SST39LF400.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/SST39LF400.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/SST39VF040.o: ../../../Board\ Support\ Package/SST39VF040.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST39VF040.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST39VF040.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/SST39VF040.c"  -o ${OBJECTDIR}/_ext/1687970324/SST39VF040.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/SST39VF040.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/SST39VF040.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o: ../../../Board\ Support\ Package/MCHP25LC256.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/MCHP25LC256.c"  -o ${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/SST26VF016B.o: SST26VF016B.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SST26VF016B.o.d 
	@${RM} ${OBJECTDIR}/SST26VF016B.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  SST26VF016B.c  -o ${OBJECTDIR}/SST26VF016B.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SST26VF016B.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/SST26VF016B.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1860783399/TimeDelay.o: ../../../Microchip/Common/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1860783399" 
	@${RM} ${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1860783399/TimeDelay.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Common/TimeDelay.c  -o ${OBJECTDIR}/_ext/1860783399/TimeDelay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/gfxepmp.o: ../../../Microchip/Graphics/Drivers/gfxepmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/gfxepmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/gfxepmp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/gfxepmp.c  -o ${OBJECTDIR}/_ext/1919882987/gfxepmp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/gfxepmp.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/gfxepmp.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o: ../../../Microchip/Graphics/Drivers/mchpGfxDrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/mchpGfxDrv.c  -o ${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o: ../../../Microchip/Graphics/Drivers/mchpGfxLCC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/mchpGfxLCC.c  -o ${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/TCON_Custom.o: ../../../Microchip/Graphics/Drivers/TCON_Custom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_Custom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_Custom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/TCON_Custom.c  -o ${OBJECTDIR}/_ext/1919882987/TCON_Custom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/TCON_Custom.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/TCON_Custom.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o: ../../../Microchip/Graphics/Drivers/TCON_HX8238.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/TCON_HX8238.c  -o ${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o: ../../../Microchip/Graphics/Drivers/TCON_HX8257.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/TCON_HX8257.c  -o ${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o: ../../../Microchip/Graphics/Drivers/TCON_SSD1289.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/TCON_SSD1289.c  -o ${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/drvTFT001.o: ../../../Microchip/Graphics/Drivers/drvTFT001.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/drvTFT001.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/drvTFT001.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/drvTFT001.c  -o ${OBJECTDIR}/_ext/1919882987/drvTFT001.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/drvTFT001.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/drvTFT001.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/drvTFT002.o: ../../../Microchip/Graphics/Drivers/drvTFT002.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/drvTFT002.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/drvTFT002.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/drvTFT002.c  -o ${OBJECTDIR}/_ext/1919882987/drvTFT002.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/drvTFT002.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/drvTFT002.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/HIT1270.o: ../../../Microchip/Graphics/Drivers/HIT1270.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/HIT1270.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/HIT1270.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/HIT1270.c  -o ${OBJECTDIR}/_ext/1919882987/HIT1270.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/HIT1270.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/HIT1270.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/HX8347.o: ../../../Microchip/Graphics/Drivers/HX8347.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/HX8347.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/HX8347.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/HX8347.c  -o ${OBJECTDIR}/_ext/1919882987/HX8347.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/HX8347.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/HX8347.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/S1D13517.o: ../../../Microchip/Graphics/Drivers/S1D13517.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/S1D13517.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/S1D13517.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/S1D13517.c  -o ${OBJECTDIR}/_ext/1919882987/S1D13517.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/S1D13517.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/S1D13517.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o: ../../../Microchip/Graphics/Drivers/SH1101A_SSD1303.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/SH1101A_SSD1303.c  -o ${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/SSD1339.o: ../../../Microchip/Graphics/Drivers/SSD1339.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SSD1339.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SSD1339.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/SSD1339.c  -o ${OBJECTDIR}/_ext/1919882987/SSD1339.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/SSD1339.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/SSD1339.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/SSD1926.o: ../../../Microchip/Graphics/Drivers/SSD1926.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SSD1926.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SSD1926.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/SSD1926.c  -o ${OBJECTDIR}/_ext/1919882987/SSD1926.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/SSD1926.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/SSD1926.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/ST7529.o: ../../../Microchip/Graphics/Drivers/ST7529.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/ST7529.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/ST7529.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/ST7529.c  -o ${OBJECTDIR}/_ext/1919882987/ST7529.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/ST7529.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/ST7529.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/UC1610.o: ../../../Microchip/Graphics/Drivers/UC1610.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/UC1610.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/UC1610.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/UC1610.c  -o ${OBJECTDIR}/_ext/1919882987/UC1610.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/UC1610.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/UC1610.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/GOL.o: ../../../Microchip/Graphics/GOL.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOL.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOL.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/GOL.c  -o ${OBJECTDIR}/_ext/803622919/GOL.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/GOL.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/GOL.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/GOLFontDefault.o: ../../../Microchip/Graphics/GOLFontDefault.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOLFontDefault.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOLFontDefault.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/GOLFontDefault.c  -o ${OBJECTDIR}/_ext/803622919/GOLFontDefault.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/GOLFontDefault.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/GOLFontDefault.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o: ../../../Microchip/Graphics/GOLSchemeDefault.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/GOLSchemeDefault.c  -o ${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Palette.o: ../../../Microchip/Graphics/Palette.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Palette.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Palette.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Palette.c  -o ${OBJECTDIR}/_ext/803622919/Palette.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Palette.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Palette.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Primitive.o: ../../../Microchip/Graphics/Primitive.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Primitive.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Primitive.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Primitive.c  -o ${OBJECTDIR}/_ext/803622919/Primitive.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Primitive.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Primitive.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Button.o: ../../../Microchip/Graphics/Button.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Button.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Button.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Button.c  -o ${OBJECTDIR}/_ext/803622919/Button.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Button.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Button.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/CheckBox.o: ../../../Microchip/Graphics/CheckBox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/CheckBox.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/CheckBox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/CheckBox.c  -o ${OBJECTDIR}/_ext/803622919/CheckBox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/CheckBox.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/CheckBox.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Meter.o: ../../../Microchip/Graphics/Meter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Meter.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Meter.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Meter.c  -o ${OBJECTDIR}/_ext/803622919/Meter.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Meter.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Meter.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/DigitalMeter.o: ../../../Microchip/Graphics/DigitalMeter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/DigitalMeter.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/DigitalMeter.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/DigitalMeter.c  -o ${OBJECTDIR}/_ext/803622919/DigitalMeter.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/DigitalMeter.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/DigitalMeter.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Chart.o: ../../../Microchip/Graphics/Chart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Chart.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Chart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Chart.c  -o ${OBJECTDIR}/_ext/803622919/Chart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Chart.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Chart.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/RadioButton.o: ../../../Microchip/Graphics/RadioButton.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/RadioButton.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/RadioButton.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/RadioButton.c  -o ${OBJECTDIR}/_ext/803622919/RadioButton.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/RadioButton.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/RadioButton.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/ListBox.o: ../../../Microchip/Graphics/ListBox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/ListBox.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/ListBox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/ListBox.c  -o ${OBJECTDIR}/_ext/803622919/ListBox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/ListBox.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/ListBox.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/GroupBox.o: ../../../Microchip/Graphics/GroupBox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/GroupBox.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/GroupBox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/GroupBox.c  -o ${OBJECTDIR}/_ext/803622919/GroupBox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/GroupBox.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/GroupBox.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/EditBox.o: ../../../Microchip/Graphics/EditBox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/EditBox.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/EditBox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/EditBox.c  -o ${OBJECTDIR}/_ext/803622919/EditBox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/EditBox.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/EditBox.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Window.o: ../../../Microchip/Graphics/Window.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Window.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Window.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Window.c  -o ${OBJECTDIR}/_ext/803622919/Window.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Window.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Window.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Slider.o: ../../../Microchip/Graphics/Slider.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Slider.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Slider.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Slider.c  -o ${OBJECTDIR}/_ext/803622919/Slider.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Slider.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Slider.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/StaticText.o: ../../../Microchip/Graphics/StaticText.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/StaticText.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/StaticText.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/StaticText.c  -o ${OBJECTDIR}/_ext/803622919/StaticText.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/StaticText.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/StaticText.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/RoundDial.o: ../../../Microchip/Graphics/RoundDial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/RoundDial.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/RoundDial.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/RoundDial.c  -o ${OBJECTDIR}/_ext/803622919/RoundDial.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/RoundDial.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/RoundDial.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Picture.o: ../../../Microchip/Graphics/Picture.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Picture.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Picture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Picture.c  -o ${OBJECTDIR}/_ext/803622919/Picture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Picture.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Picture.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/ProgressBar.o: ../../../Microchip/Graphics/ProgressBar.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/ProgressBar.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/ProgressBar.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/ProgressBar.c  -o ${OBJECTDIR}/_ext/803622919/ProgressBar.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/ProgressBar.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/ProgressBar.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/TextEntry.o: ../../../Microchip/Graphics/TextEntry.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/TextEntry.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/TextEntry.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/TextEntry.c  -o ${OBJECTDIR}/_ext/803622919/TextEntry.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/TextEntry.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/TextEntry.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Main.o: Main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Main.o.d 
	@${RM} ${OBJECTDIR}/Main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Main.c  -o ${OBJECTDIR}/Main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/DRV_SPI_SMCPU.o: DRV_SPI_SMCPU.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/DRV_SPI_SMCPU.o.d 
	@${RM} ${OBJECTDIR}/DRV_SPI_SMCPU.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  DRV_SPI_SMCPU.c  -o ${OBJECTDIR}/DRV_SPI_SMCPU.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/DRV_SPI_SMCPU.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/DRV_SPI_SMCPU.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/CRC16.o: CRC16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/CRC16.o.d 
	@${RM} ${OBJECTDIR}/CRC16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  CRC16.c  -o ${OBJECTDIR}/CRC16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/CRC16.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/CRC16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Pattern/GDD_Screens.o: Pattern/GDD_Screens.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Pattern" 
	@${RM} ${OBJECTDIR}/Pattern/GDD_Screens.o.d 
	@${RM} ${OBJECTDIR}/Pattern/GDD_Screens.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Pattern/GDD_Screens.c  -o ${OBJECTDIR}/Pattern/GDD_Screens.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Pattern/GDD_Screens.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Pattern/GDD_Screens.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Pattern/GDD_Resource.o: Pattern/GDD_Resource.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Pattern" 
	@${RM} ${OBJECTDIR}/Pattern/GDD_Resource.o.d 
	@${RM} ${OBJECTDIR}/Pattern/GDD_Resource.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Pattern/GDD_Resource.c  -o ${OBJECTDIR}/Pattern/GDD_Resource.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Pattern/GDD_Resource.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Pattern/GDD_Resource.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o: Pattern/GDD_X_Event_Handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Pattern" 
	@${RM} ${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o.d 
	@${RM} ${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Pattern/GDD_X_Event_Handler.c  -o ${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1687970324/TouchScreen.o: ../../../Board\ Support\ Package/TouchScreen.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreen.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreen.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/TouchScreen.c"  -o ${OBJECTDIR}/_ext/1687970324/TouchScreen.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/TouchScreen.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/TouchScreen.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o: ../../../Board\ Support\ Package/TouchScreenAR1020.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/TouchScreenAR1020.c"  -o ${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/TouchScreenAR1020.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o: ../../../Board\ Support\ Package/TouchScreenResistive.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/TouchScreenResistive.c"  -o ${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/TouchScreenResistive.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/Beep.o: ../../../Board\ Support\ Package/Beep.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/Beep.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/Beep.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/Beep.c"  -o ${OBJECTDIR}/_ext/1687970324/Beep.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/Beep.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/Beep.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/SST25VF016.o: ../../../Board\ Support\ Package/SST25VF016.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST25VF016.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST25VF016.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/SST25VF016.c"  -o ${OBJECTDIR}/_ext/1687970324/SST25VF016.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/SST25VF016.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/SST25VF016.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/cpld.o: ../../../Board\ Support\ Package/cpld.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/cpld.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/cpld.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/cpld.c"  -o ${OBJECTDIR}/_ext/1687970324/cpld.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/cpld.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/cpld.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/drv_spi.o: ../../../Board\ Support\ Package/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/drv_spi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/drv_spi.c"  -o ${OBJECTDIR}/_ext/1687970324/drv_spi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/drv_spi.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/drv_spi.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/SST39LF400.o: ../../../Board\ Support\ Package/SST39LF400.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST39LF400.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST39LF400.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/SST39LF400.c"  -o ${OBJECTDIR}/_ext/1687970324/SST39LF400.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/SST39LF400.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/SST39LF400.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/SST39VF040.o: ../../../Board\ Support\ Package/SST39VF040.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST39VF040.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/SST39VF040.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/SST39VF040.c"  -o ${OBJECTDIR}/_ext/1687970324/SST39VF040.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/SST39VF040.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/SST39VF040.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o: ../../../Board\ Support\ Package/MCHP25LC256.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1687970324" 
	@${RM} ${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o.d 
	@${RM} ${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../Board Support Package/MCHP25LC256.c"  -o ${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1687970324/MCHP25LC256.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/SST26VF016B.o: SST26VF016B.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SST26VF016B.o.d 
	@${RM} ${OBJECTDIR}/SST26VF016B.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  SST26VF016B.c  -o ${OBJECTDIR}/SST26VF016B.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SST26VF016B.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/SST26VF016B.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1860783399/TimeDelay.o: ../../../Microchip/Common/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1860783399" 
	@${RM} ${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1860783399/TimeDelay.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Common/TimeDelay.c  -o ${OBJECTDIR}/_ext/1860783399/TimeDelay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/gfxepmp.o: ../../../Microchip/Graphics/Drivers/gfxepmp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/gfxepmp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/gfxepmp.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/gfxepmp.c  -o ${OBJECTDIR}/_ext/1919882987/gfxepmp.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/gfxepmp.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/gfxepmp.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o: ../../../Microchip/Graphics/Drivers/mchpGfxDrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/mchpGfxDrv.c  -o ${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/mchpGfxDrv.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o: ../../../Microchip/Graphics/Drivers/mchpGfxLCC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/mchpGfxLCC.c  -o ${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/mchpGfxLCC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/TCON_Custom.o: ../../../Microchip/Graphics/Drivers/TCON_Custom.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_Custom.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_Custom.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/TCON_Custom.c  -o ${OBJECTDIR}/_ext/1919882987/TCON_Custom.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/TCON_Custom.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/TCON_Custom.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o: ../../../Microchip/Graphics/Drivers/TCON_HX8238.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/TCON_HX8238.c  -o ${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/TCON_HX8238.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o: ../../../Microchip/Graphics/Drivers/TCON_HX8257.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/TCON_HX8257.c  -o ${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/TCON_HX8257.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o: ../../../Microchip/Graphics/Drivers/TCON_SSD1289.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/TCON_SSD1289.c  -o ${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/TCON_SSD1289.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/drvTFT001.o: ../../../Microchip/Graphics/Drivers/drvTFT001.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/drvTFT001.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/drvTFT001.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/drvTFT001.c  -o ${OBJECTDIR}/_ext/1919882987/drvTFT001.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/drvTFT001.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/drvTFT001.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/drvTFT002.o: ../../../Microchip/Graphics/Drivers/drvTFT002.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/drvTFT002.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/drvTFT002.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/drvTFT002.c  -o ${OBJECTDIR}/_ext/1919882987/drvTFT002.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/drvTFT002.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/drvTFT002.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/HIT1270.o: ../../../Microchip/Graphics/Drivers/HIT1270.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/HIT1270.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/HIT1270.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/HIT1270.c  -o ${OBJECTDIR}/_ext/1919882987/HIT1270.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/HIT1270.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/HIT1270.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/HX8347.o: ../../../Microchip/Graphics/Drivers/HX8347.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/HX8347.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/HX8347.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/HX8347.c  -o ${OBJECTDIR}/_ext/1919882987/HX8347.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/HX8347.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/HX8347.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/S1D13517.o: ../../../Microchip/Graphics/Drivers/S1D13517.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/S1D13517.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/S1D13517.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/S1D13517.c  -o ${OBJECTDIR}/_ext/1919882987/S1D13517.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/S1D13517.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/S1D13517.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o: ../../../Microchip/Graphics/Drivers/SH1101A_SSD1303.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/SH1101A_SSD1303.c  -o ${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/SH1101A_SSD1303.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/SSD1339.o: ../../../Microchip/Graphics/Drivers/SSD1339.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SSD1339.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SSD1339.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/SSD1339.c  -o ${OBJECTDIR}/_ext/1919882987/SSD1339.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/SSD1339.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/SSD1339.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/SSD1926.o: ../../../Microchip/Graphics/Drivers/SSD1926.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SSD1926.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/SSD1926.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/SSD1926.c  -o ${OBJECTDIR}/_ext/1919882987/SSD1926.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/SSD1926.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/SSD1926.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/ST7529.o: ../../../Microchip/Graphics/Drivers/ST7529.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/ST7529.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/ST7529.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/ST7529.c  -o ${OBJECTDIR}/_ext/1919882987/ST7529.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/ST7529.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/ST7529.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1919882987/UC1610.o: ../../../Microchip/Graphics/Drivers/UC1610.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1919882987" 
	@${RM} ${OBJECTDIR}/_ext/1919882987/UC1610.o.d 
	@${RM} ${OBJECTDIR}/_ext/1919882987/UC1610.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Drivers/UC1610.c  -o ${OBJECTDIR}/_ext/1919882987/UC1610.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1919882987/UC1610.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1919882987/UC1610.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/GOL.o: ../../../Microchip/Graphics/GOL.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOL.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOL.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/GOL.c  -o ${OBJECTDIR}/_ext/803622919/GOL.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/GOL.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/GOL.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/GOLFontDefault.o: ../../../Microchip/Graphics/GOLFontDefault.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOLFontDefault.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOLFontDefault.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/GOLFontDefault.c  -o ${OBJECTDIR}/_ext/803622919/GOLFontDefault.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/GOLFontDefault.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/GOLFontDefault.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o: ../../../Microchip/Graphics/GOLSchemeDefault.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/GOLSchemeDefault.c  -o ${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/GOLSchemeDefault.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Palette.o: ../../../Microchip/Graphics/Palette.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Palette.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Palette.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Palette.c  -o ${OBJECTDIR}/_ext/803622919/Palette.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Palette.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Palette.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Primitive.o: ../../../Microchip/Graphics/Primitive.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Primitive.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Primitive.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Primitive.c  -o ${OBJECTDIR}/_ext/803622919/Primitive.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Primitive.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Primitive.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Button.o: ../../../Microchip/Graphics/Button.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Button.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Button.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Button.c  -o ${OBJECTDIR}/_ext/803622919/Button.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Button.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Button.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/CheckBox.o: ../../../Microchip/Graphics/CheckBox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/CheckBox.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/CheckBox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/CheckBox.c  -o ${OBJECTDIR}/_ext/803622919/CheckBox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/CheckBox.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/CheckBox.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Meter.o: ../../../Microchip/Graphics/Meter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Meter.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Meter.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Meter.c  -o ${OBJECTDIR}/_ext/803622919/Meter.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Meter.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Meter.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/DigitalMeter.o: ../../../Microchip/Graphics/DigitalMeter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/DigitalMeter.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/DigitalMeter.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/DigitalMeter.c  -o ${OBJECTDIR}/_ext/803622919/DigitalMeter.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/DigitalMeter.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/DigitalMeter.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Chart.o: ../../../Microchip/Graphics/Chart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Chart.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Chart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Chart.c  -o ${OBJECTDIR}/_ext/803622919/Chart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Chart.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Chart.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/RadioButton.o: ../../../Microchip/Graphics/RadioButton.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/RadioButton.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/RadioButton.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/RadioButton.c  -o ${OBJECTDIR}/_ext/803622919/RadioButton.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/RadioButton.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/RadioButton.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/ListBox.o: ../../../Microchip/Graphics/ListBox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/ListBox.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/ListBox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/ListBox.c  -o ${OBJECTDIR}/_ext/803622919/ListBox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/ListBox.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/ListBox.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/GroupBox.o: ../../../Microchip/Graphics/GroupBox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/GroupBox.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/GroupBox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/GroupBox.c  -o ${OBJECTDIR}/_ext/803622919/GroupBox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/GroupBox.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/GroupBox.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/EditBox.o: ../../../Microchip/Graphics/EditBox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/EditBox.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/EditBox.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/EditBox.c  -o ${OBJECTDIR}/_ext/803622919/EditBox.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/EditBox.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/EditBox.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Window.o: ../../../Microchip/Graphics/Window.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Window.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Window.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Window.c  -o ${OBJECTDIR}/_ext/803622919/Window.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Window.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Window.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Slider.o: ../../../Microchip/Graphics/Slider.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Slider.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Slider.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Slider.c  -o ${OBJECTDIR}/_ext/803622919/Slider.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Slider.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Slider.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/StaticText.o: ../../../Microchip/Graphics/StaticText.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/StaticText.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/StaticText.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/StaticText.c  -o ${OBJECTDIR}/_ext/803622919/StaticText.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/StaticText.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/StaticText.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/RoundDial.o: ../../../Microchip/Graphics/RoundDial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/RoundDial.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/RoundDial.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/RoundDial.c  -o ${OBJECTDIR}/_ext/803622919/RoundDial.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/RoundDial.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/RoundDial.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/Picture.o: ../../../Microchip/Graphics/Picture.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/Picture.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/Picture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/Picture.c  -o ${OBJECTDIR}/_ext/803622919/Picture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/Picture.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/Picture.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/ProgressBar.o: ../../../Microchip/Graphics/ProgressBar.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/ProgressBar.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/ProgressBar.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/ProgressBar.c  -o ${OBJECTDIR}/_ext/803622919/ProgressBar.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/ProgressBar.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/ProgressBar.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/803622919/TextEntry.o: ../../../Microchip/Graphics/TextEntry.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/803622919" 
	@${RM} ${OBJECTDIR}/_ext/803622919/TextEntry.o.d 
	@${RM} ${OBJECTDIR}/_ext/803622919/TextEntry.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Graphics/TextEntry.c  -o ${OBJECTDIR}/_ext/803622919/TextEntry.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/803622919/TextEntry.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/803622919/TextEntry.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Main.o: Main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Main.o.d 
	@${RM} ${OBJECTDIR}/Main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Main.c  -o ${OBJECTDIR}/Main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Main.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/DRV_SPI_SMCPU.o: DRV_SPI_SMCPU.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/DRV_SPI_SMCPU.o.d 
	@${RM} ${OBJECTDIR}/DRV_SPI_SMCPU.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  DRV_SPI_SMCPU.c  -o ${OBJECTDIR}/DRV_SPI_SMCPU.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/DRV_SPI_SMCPU.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/DRV_SPI_SMCPU.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/CRC16.o: CRC16.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/CRC16.o.d 
	@${RM} ${OBJECTDIR}/CRC16.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  CRC16.c  -o ${OBJECTDIR}/CRC16.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/CRC16.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/CRC16.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Pattern/GDD_Screens.o: Pattern/GDD_Screens.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Pattern" 
	@${RM} ${OBJECTDIR}/Pattern/GDD_Screens.o.d 
	@${RM} ${OBJECTDIR}/Pattern/GDD_Screens.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Pattern/GDD_Screens.c  -o ${OBJECTDIR}/Pattern/GDD_Screens.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Pattern/GDD_Screens.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Pattern/GDD_Screens.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Pattern/GDD_Resource.o: Pattern/GDD_Resource.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Pattern" 
	@${RM} ${OBJECTDIR}/Pattern/GDD_Resource.o.d 
	@${RM} ${OBJECTDIR}/Pattern/GDD_Resource.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Pattern/GDD_Resource.c  -o ${OBJECTDIR}/Pattern/GDD_Resource.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Pattern/GDD_Resource.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Pattern/GDD_Resource.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o: Pattern/GDD_X_Event_Handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/Pattern" 
	@${RM} ${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o.d 
	@${RM} ${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Pattern/GDD_X_Event_Handler.c  -o ${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mlarge-code -mlarge-data -O1 -I"../../../Board Support Package" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Graphics" -I"." -I"Pattern" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Pattern/GDD_X_Event_Handler.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Pattern.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Pattern.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x800:0x81B -mreserve=data@0x81C:0x81D -mreserve=data@0x81E:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x827 -mreserve=data@0x82A:0x84F   -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,$(MP_LINKER_FILE_OPTION),--heap=10000,--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Pattern.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Pattern.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--heap=10000,--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Pattern.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
