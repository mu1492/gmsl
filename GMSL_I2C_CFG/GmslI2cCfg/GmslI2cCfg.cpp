/*
GmslI2cCfg.cpp

This file contains the sources for GMSL I2C configurator.


Copyright (c) 2022, Mihai Ursu

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "GmslI2cCfg.h"
#include "Platform.h"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <utility>

#include <termios.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/time.h>

//!************************************************************************
//! Constructor
//!************************************************************************
GmslI2cCfg::GmslI2cCfg()
    : mSequenceIndex( SEQUENCE_NONE )
{
    Platform* platformInstance = Platform::getInstance();

    if( platformInstance )
    {
        platformInstance->setDes( Platform::DESERIALIZER_MAX96724 );
        platformInstance->updateDesFromKernel();

        platformInstance->setSer( Platform::SERIALIZER_MAX96717 );

        createSequencesMap();
        int sequenceOption = SEQUENCE_NONE;
        std::vector<GmslAction> sequenceVec;

        uint8_t readReg = 0;
        uint8_t deviceId = 0;
        uint8_t reg5 = 0;

        do{
            sequenceVec.clear();
            printOptions();

            do{
                std::cout << "\n\n Please enter your option: ";
                std::cin >> sequenceOption;
            }while( sequenceOption > SEQUENCE_COUNT );

            std::cout << " " << mSequencesMap.at( sequenceOption ) << "\n";

            sequenceVec = createSequence( static_cast<Sequence>( sequenceOption ) );
            size_t seqLen = sequenceVec.size();

            if( seqLen )
            {
                ///////////////////////////////
                // pre-running actions
                ///////////////////////////////
                switch( sequenceOption )
                {
                    case SEQUENCE_DES_MAX96724_TOGGLE_LOCK_LED:
                        {
                            const uint8_t DES_I2C_ADDRESS = platformInstance->getDesI2cSlaveAddress();
                            // save reg5 value -> it will be restored during post-running
                            GmslAction reg5SaveAction = GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_READ, MAX96724_REG_REG5, 0, 0, false, 0 );
                            platformInstance->readAction( reg5SaveAction, &reg5 );
                        }
                        break;

                    default:
                        break;
                }

                ///////////////////////////////
                // running actions
                ///////////////////////////////
                for( size_t i = 0; i < seqLen; i++ )
                {
                    std::cout << "\n Action " << i + 1 << " of " << seqLen;
                    std::fflush( stdout );

                    switch( sequenceVec.at( i ).mActionType )
                    {
                        case GmslAction::ACTION_TYPE_WRITE:
                            platformInstance->writeAction( sequenceVec.at( i ) );
                            break;

                        case GmslAction::ACTION_TYPE_WRITE_WITH_MASK:
                            platformInstance->writeWithMaskAction( sequenceVec.at( i ) );
                            break;

                        case GmslAction::ACTION_TYPE_DELAY:
                            platformInstance->delayAction( sequenceVec.at( i ) );
                            break;

                        case GmslAction::ACTION_TYPE_READ:
                            {
                                uint8_t dummyByte = 0;
                                platformInstance->readAction( sequenceVec.at( i ), &dummyByte );

                                switch( sequenceOption )
                                {
                                    case SEQUENCE_DES_MAX96724_READ_REGISTER:
                                    case SEQUENCE_SER_MAX96717_READ_REGISTER:
                                        readReg = dummyByte;
                                        break;

                                    case SEQUENCE_DES_MAX96724_READ_DEVICE_ID:
                                    case SEQUENCE_SER_MAX96717_READ_DEVICE_ID:
                                        deviceId = dummyByte;
                                        break;

                                    default:
                                        break;
                                }
                            }
                            break;

                        default:
                            break;
                    }

                    std::fflush( stdout );
                }

                ///////////////////////////////
                // post-running actions
                ///////////////////////////////
                switch( sequenceOption )
                {
                    case SEQUENCE_DES_MAX96724_READ_REGISTER:
                    case SEQUENCE_SER_MAX96717_READ_REGISTER:
                        std::cout << "\n Register value is = 0x" << std::setfill( '0' ) << std::setw( 2 ) << std::hex << static_cast<int>( readReg ) << std::dec;
                        break;

                    case SEQUENCE_DES_MAX96724_READ_DEVICE_ID:
                    case SEQUENCE_SER_MAX96717_READ_DEVICE_ID:
                        std::cout << "\n Device ID = 0x" << std::setfill( '0' ) << std::setw( 2 ) << std::hex << static_cast<int>( deviceId ) << std::dec;
                        break;

                    case SEQUENCE_DES_MAX96724_TOGGLE_LOCK_LED:
                        {
                            const uint8_t DES_I2C_ADDRESS = platformInstance->getDesI2cSlaveAddress();
                            // restore reg5 value
                            GmslAction reg5RestoreAction = GmslAction( DES_I2C_ADDRESS, MAX96724_REG_REG5, reg5, false );
                            platformInstance->writeAction( reg5RestoreAction );
                        }
                        break;

                    case SEQUENCE_CAM_AR0234_SER_MAX96717_DES_MAX96724:
                        {
                            const uint8_t CAM_I2C_ADDRESS = 0x10;
                            bool exitCameraOption = false;

                            static uint32_t aGain = 0x000d;
                            const uint8_t A_GAIN_STEP = 10;

                            std::cout << "\n\n AR0234 camera submenu";
                            std::cout << "\n\t 'c' - 100% color bars pattern";
                            std::cout << "\n\t 'f' - fade-to-grey color bars pattern";
                            std::cout << "\n\t 'r' - 75% red color pattern";
                            std::cout << "\n\t 'g' - 75% green color pattern";
                            std::cout << "\n\t 'b' - 75% blue color pattern";
                            std::cout << "\n\t 'w' - walking 1s pattern";
                            std::cout << "\n\t 'v' - video feed";
                            std::cout << "\n\t '+' - increase analog gain";
                            std::cout << "\n\t '-' - decrease analog gain";
                            std::cout << "\n\t ESC - exit this submenu";
                            std::cout << "\n";

                            while( !exitCameraOption )
                            {
                                if( kbhit() )
                                {
                                    switch( tolower( getch() ) )
                                    {
                                        case 27:
                                            exitCameraOption = true;
                                            std::cout << "\n Exit from camera menu\n";
                                            break;

                                        case 'c':
                                            {
                                                // 100% color bars pattern
                                                GmslAction action = GmslAction( CAM_I2C_ADDRESS, 0x3070, 0x0002, true );
                                                platformInstance->writeAction( action );
                                            }
                                            break;

                                        case 'f':
                                            {
                                                // fade-to-grey color bars pattern
                                                GmslAction action = GmslAction( CAM_I2C_ADDRESS, 0x3070, 0x0003, true );
                                                platformInstance->writeAction( action );
                                            }
                                            break;

                                        case 'r':
                                            {
                                                // 75% red solid color pattern
                                                GmslAction action = GmslAction( CAM_I2C_ADDRESS, 0x3072, 0xbfff, true );
                                                platformInstance->writeAction( action );
                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3074, 0x0000, true );
                                                platformInstance->writeAction( action );
                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3078, 0x0000, true );
                                                platformInstance->writeAction( action );
                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3076, 0x0000, true );
                                                platformInstance->writeAction( action );

                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3070, 0x0001, true );
                                                platformInstance->writeAction( action );
                                            }
                                            break;

                                        case 'g':
                                            {
                                                // 75% green solid color pattern
                                                GmslAction action = GmslAction( CAM_I2C_ADDRESS, 0x3072, 0x0000, true );
                                                platformInstance->writeAction( action );
                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3074, 0xbfff, true );
                                                platformInstance->writeAction( action );
                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3078, 0xbfff, true );
                                                platformInstance->writeAction( action );
                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3076, 0x0000, true );
                                                platformInstance->writeAction( action );

                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3070, 0x0001, true );
                                                platformInstance->writeAction( action );
                                            }
                                            break;

                                        case 'b':
                                            {
                                                // 75% blue solid color pattern
                                                GmslAction action = GmslAction( CAM_I2C_ADDRESS, 0x3072, 0x0000, true );
                                                platformInstance->writeAction( action );
                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3074, 0x0000, true );
                                                platformInstance->writeAction( action );
                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3078, 0x0000, true );
                                                platformInstance->writeAction( action );
                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3076, 0xbfff, true );
                                                platformInstance->writeAction( action );

                                                action = GmslAction( CAM_I2C_ADDRESS, 0x3070, 0x0001, true );
                                                platformInstance->writeAction( action );
                                            }
                                            break;

                                        case 'w':
                                            {
                                                // walking 1s pattern
                                                GmslAction action = GmslAction( CAM_I2C_ADDRESS, 0x3070, 0x0100, true ); 
                                                platformInstance->writeAction( action );
                                            }
                                            break;

                                        case 'v':
                                            {
                                                // no pattern / video feed
                                                GmslAction action = GmslAction( CAM_I2C_ADDRESS, 0x3070, 0x0000, true ); 
                                                platformInstance->writeAction( action );
                                            }
                                            break;

                                        case '+':
                                            {
                                                // increase gain
                                                if( aGain < 0xffff - A_GAIN_STEP )
                                                {
                                                    aGain += A_GAIN_STEP;
                                                }

                                                GmslAction action = GmslAction( CAM_I2C_ADDRESS, 0x3060, aGain, true );
                                                platformInstance->writeAction( action );
                                            }
                                            break;

                                        case '-':
                                            {
                                                // decrease gain
                                                if( aGain > A_GAIN_STEP )
                                                {
                                                    aGain -= A_GAIN_STEP;
                                                }

                                                GmslAction action = GmslAction( CAM_I2C_ADDRESS, 0x3060, aGain, true );
                                                platformInstance->writeAction( action );
                                            }
                                            break;

                                        default:
                                            break;
                                    }
                                }
                            }
                        }
                        break;

                    default:
                        break;
                }

                std::cout << "\n";
            }
            else
            {
                std::cout << "\nNo GMSL sequence was created.\n\n";
            }

        }while( SEQUENCE_NONE != sequenceOption );
    }
}


//!************************************************************************
//! Create a sequence vector for a specified GMSL configuration
//!
//! @returns: the sequence vector
//!************************************************************************
std::vector<GmslAction> GmslI2cCfg::createSequence
    (
    const Sequence aIndex            //!< sequence index
    )
{
    std::vector<GmslAction> sequenceVec;
    Platform* platformInstance = Platform::getInstance();

    if( platformInstance )
    {
        switch( aIndex )
        {
            case SEQUENCE_DES_MAX96724_READ_REGISTER:
                {
                    const uint8_t DES_I2C_ADDRESS = platformInstance->getDesI2cSlaveAddress();
                    uint16_t desRegAddress = getUserRegAddress();
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_READ, desRegAddress, 0, 0, false, 0 ) );
                }
                break;

            case SEQUENCE_DES_MAX96724_READ_DEVICE_ID:
                {
                    const uint8_t DES_I2C_ADDRESS = platformInstance->getDesI2cSlaveAddress();
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_READ, MAX96724_REG_REG13, 0, 0, false, 0 ) );
                }
                break;

            case SEQUENCE_DES_MAX96724_TOGGLE_LOCK_LED:
                {
                    const uint8_t DES_I2C_ADDRESS = platformInstance->getDesI2cSlaveAddress();

                    for( int i = 0; i < 5; i++ )
                    {
                        // lock LED -> off
                        sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_REG5, 0x80, false ) );
                        // delay 50 ms
                        sequenceVec.push_back( GmslAction( 50000 ) );

                        // lock LED -> on
                        sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_REG5, 0x00, false ) );
                        // delay 50 ms
                        sequenceVec.push_back( GmslAction( 50000 ) );
                    }
                }
                break;

            case SEQUENCE_DES_MAX96724_RESET:
                {
                    const uint8_t DES_I2C_ADDRESS = platformInstance->getDesI2cSlaveAddress();

                    // see t_LOCK2 in datasheet (45-60ms)
                    const uint8_t MAX96724_DELAY_MS = 60;

                    // reset
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_PWR1, 0x40, 0x40, false, 0 ) ); // RESET_ALL
                    sequenceVec.push_back( GmslAction( MAX96724_DELAY_MS * 1000 ) );

                    // make sure MIPI is disabled
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_MIPI_PHY0, 0x80, 0x00, false, 0 ) ); // FORCE_CSI_OUT_EN => disable
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_BACKTOP12_OF_BACKTOP, 0x02, 0x00, false, 0 ) ); // CSI_OUT_EN => disable

                    // configure MIPI
                    // select MIPI output as 4 ports with 2 data lanes each
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_MIPI_PHY0, 0x01, false ) ); // phy_4x2

                    // configure 2 data lane for PHY2
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_MIPI_TX10_OF_MIPI_TX2, 0xc0, 0x40, false, 0 ) ); // CSI2_LANE_CNT

                    // configure lane mapping for PHY3 (bits 7:4) and PHY2 (bits 3:0)
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_MIPI_PHY4, 0xe4, false ) ); // phy3_lane_map, phy2_lane_map

                    // enable PHY2 only
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_MIPI_PHY2, 0xf0, 0x40, false, 0 ) ); // phy_Stdby_n

                    // enable MIPI
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_BACKTOP12_OF_BACKTOP, 0x02, 0x02, false, 0 ) ); // CSI_OUT_EN => enable
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_MIPI_PHY0, 0x80, 0x80, false, 0 ) ); // FORCE_CSI_OUT_EN => enable
                }
                break;

            case SEQUENCE_DES_MAX96724_SET_VIDEO_FMT_CODE_RGB888:
                platformInstance->setDesKernelParamI2cCmd( Platform::KERNEL_PARAMS_CMD_SET_VIDEO_FMT_CODE_RGB888 );
                break;

            case SEQUENCE_DES_MAX96724_SET_VIDEO_FMT_CODE_RAW10:
                platformInstance->setDesKernelParamI2cCmd( Platform::KERNEL_PARAMS_CMD_SET_VIDEO_FMT_CODE_RAW10 );
                break;

            case SEQUENCE_DES_MAX96724_KERNEL_PATTERN_COLORBARS_ON:
                platformInstance->setDesKernelParamI2cCmd( Platform::KERNEL_PARAMS_CMD_START_PATTERN_COLORBARS );
                break;

            case SEQUENCE_DES_MAX96724_KERNEL_PATTERN_OFF:
                platformInstance->setDesKernelParamI2cCmd( Platform::KERNEL_PARAMS_CMD_STOP_PATTERN );
                break;

            case SEQUENCE_DES_MAX96724_USERSPACE_PATTERN_COLORBARS_ON:
                {
                    const uint8_t DES_I2C_ADDRESS = platformInstance->getDesI2cSlaveAddress();

                    /////////////////////////////////////////
                    // given values for 1920x1080 @ 30 fps
                    /////////////////////////////////////////
                    const uint16_t H_RES    = 1920;     // lines
                    const uint16_t H_FP     = 88;       // PCLKs
                    const uint16_t H_SYNCW  = 44;       // PCLKs
                    const uint16_t H_BP     = 148;      // PCLKs

                    const uint16_t V_RES    = 1080;     // lines
                    const uint16_t V_FP     = 4;        // lines
                    const uint16_t V_SYNCW  = 5;        // lines
                    const uint16_t V_BP     = 36;       // lines

                    const uint32_t VS_DLY   = 0;        // PCLKs
                    const uint32_t HS_DLY   = 0;        // PCLKs

                    const uint8_t GRAD_INC = 4;

                    /////////////////////////////////////////
                    // calculated values
                    /////////////////////////////////////////
                    const uint32_t H_TOTAL = H_RES + H_FP + H_SYNCW + H_BP;             //    2200
                    const uint16_t V_TOTAL = V_RES + V_FP + V_SYNCW + V_BP;             //    1125
                    const uint32_t VS_HIGH = V_SYNCW * H_TOTAL;                         //   11000
                    const uint32_t VS_LOW = ( V_RES + V_FP + V_BP ) * H_TOTAL;          // 2464000
                    const uint16_t HS_LOW = H_RES + H_FP + H_BP;                        //    2156
                    const uint32_t V2D = ( V_SYNCW + V_BP ) * H_TOTAL + H_SYNCW + H_BP; //   90392
                    const uint16_t DE_LOW = H_FP + H_SYNCW + H_BP;                      //     280

                    // variables
                    uint8_t highByte = 0;
                    uint8_t midByte = 0;
                    uint8_t lowByte = 0;

                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_PATGEN_0, 0x04, 0x00, false, 0 ) );   // do not invert DE
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_PATGEN_0, 0x08, 0x00, false, 0 ) );   // do not invert HS
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_PATGEN_0, 0x10, 0x10, false, 0 ) );   // invert VS

                    getHighMidLow( VS_DLY, &highByte, &midByte, &lowByte );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_VS_DLY_2,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_VS_DLY_2 + 1, midByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_VS_DLY_2 + 2, lowByte, false ) );

                    getHighMidLow( VS_HIGH, &highByte, &midByte, &lowByte );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_VS_HIGH_2,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_VS_HIGH_2 + 1, midByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_VS_HIGH_2 + 2, lowByte, false ) );

                    getHighMidLow( VS_LOW, &highByte, &midByte, &lowByte );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_VS_LOW_2,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_VS_LOW_2 + 1, midByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_VS_LOW_2 + 2, lowByte, false ) );

                    getHighMidLow( HS_DLY, &highByte, &midByte, &lowByte );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_V2H_2,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_V2H_2 + 1, midByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_V2H_2 + 2, lowByte, false ) );

                    getHighLow( H_SYNCW, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_HS_HIGH_1,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_HS_HIGH_1 + 1, lowByte, false ) );

                    getHighLow( HS_LOW, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_HS_LOW_1,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_HS_LOW_1 + 1, lowByte, false ) );

                    getHighLow( V_TOTAL, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_HS_CNT_1,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_HS_CNT_1 + 1, lowByte, false ) );

                    getHighMidLow( V2D, &highByte, &midByte, &lowByte );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_V2D_2,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_V2D_2 + 1, midByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_V2D_2 + 2, lowByte, false ) );

                    getHighLow( H_RES, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_DE_HIGH_1,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_DE_HIGH_1 + 1, lowByte, false ) );

                    getHighLow( DE_LOW, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_DE_LOW_1,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_DE_LOW_1 + 1, lowByte, false ) );

                    getHighLow( V_RES, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_DE_CNT_1,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_DE_CNT_1 + 1, lowByte, false ) ); 

                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_PATGEN_1, 0x30, 0x20, false, 0 ) );   // patgen_mode = colorbar

                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_GRAD_INCR, GRAD_INC, false ) );

                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_VPRBS,     0x80, 0x00, false, 0 ) );   // Pipe2 ONLY, patgen_clk_src => Value: 0
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_PCLK_FREQ, 0x03, 0x01, false, 0 ) );   // PCLK f = 75 MHz
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_MIPI_PHY0, 0x80, 0x80, false, 0 ) );   // FORCE_CSI_OUT_EN => enable
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96724_REG_PATGEN_0,  0xe3, 0xe3, false, 0 ) );   // Generate VS and HS and DE, free-running mode, do not touch inversion
                }
                break;

            case SEQUENCE_DES_MAX96724_USERSPACE_PATTERN_OFF:
                {
                    const uint8_t DES_I2C_ADDRESS = platformInstance->getDesI2cSlaveAddress();
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_PATGEN_1, 0x00, false ) );     // patgen_mode = none
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_PATGEN_0, 0x03, false ) );     // patgen reset
                }
                break;


            case SEQUENCE_SER_MAX96717_READ_REGISTER:
                {
                    const uint8_t SER_I2C_ADDRESS = platformInstance->getSerI2cSlaveAddress();
                    uint16_t serRegAddress = getUserRegAddress();
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_READ, serRegAddress, 0, 0, false, 0 ) );
                }
                break;

            case SEQUENCE_SER_MAX96717_READ_DEVICE_ID:
                {
                    const uint8_t SER_I2C_ADDRESS = platformInstance->getSerI2cSlaveAddress();
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_READ, MAX96717_REG_REG13, 0, 0, false, 0 ) );
                }
                break;

            case SEQUENCE_SER_MAX96717_RESET:
                {
                    const uint8_t SER_I2C_ADDRESS = platformInstance->getSerI2cSlaveAddress();
                    const uint8_t MAX96717_DELAY_MS = 60; // must be greater than tLOCK (typ 35 ms)

                    // reset
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_CTRL0, 0x80, 0x80, false, 0 ) ); // RESET_ALL
                    sequenceVec.push_back( GmslAction( MAX96717_DELAY_MS * 1000 ) );

                    // set 2 data lanes
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_MIPI_RX1, 0x30, 0x10, false, 0 ) ); // CTRL1_NUM_LANES

                    // increase Vreg for CMU (Clock Multiplier Unit)
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_CMU2, 0x70, 0x10, false, 0 ) ); // VREG_PFDDIV = 1.1V

                    // reset MFP0 status for camera disabled
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_GPIO_B_OF_GPIO0, 0xa0, false ) ); // pull-down buffer, push-pull driver
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_GPIO_A_OF_GPIO0, 0x00, false ) ); // drive output low (camera disabled)
                    sequenceVec.push_back( GmslAction( 10000 ) ); // delay 10 ms
                }
                break;

            case SEQUENCE_SER_MAX96717_PATTERN_CHECKERBOARD_ON:
                {
                    const uint8_t SER_I2C_ADDRESS = platformInstance->getSerI2cSlaveAddress();

                    /////////////////////////////////////////
                    // given values for 1920x1080 @ 30 fps
                    /////////////////////////////////////////
                    const uint16_t H_RES    = 1920;     // lines
                    const uint16_t H_FP     = 88;       // PCLKs
                    const uint16_t H_SYNCW  = 44;       // PCLKs
                    const uint16_t H_BP     = 148;      // PCLKs

                    const uint16_t V_RES    = 1080;     // lines
                    const uint16_t V_FP     = 4;        // lines
                    const uint16_t V_SYNCW  = 5;        // lines
                    const uint16_t V_BP     = 36;       // lines

                    const uint32_t VS_DLY   = 0;        // PCLKs
                    const uint32_t HS_DLY   = 0;        // PCLKs

                    const uint8_t CHECKER_COLOR_A_R = 0;
                    const uint8_t CHECKER_COLOR_A_G = 223;
                    const uint8_t CHECKER_COLOR_A_B = 63;

                    const uint8_t CHECKER_COLOR_B_R = 0;
                    const uint8_t CHECKER_COLOR_B_G = 0;
                    const uint8_t CHECKER_COLOR_B_B = 0;

                    const uint8_t CHECKER_WIDTH_A = 60;
                    const uint8_t CHECKER_WIDTH_B = 60;
                    const uint8_t CHECKER_HEIGHT  = 60;

                    /////////////////////////////////////////
                    // calculated values
                    /////////////////////////////////////////
                    const uint32_t H_TOTAL = H_RES + H_FP + H_SYNCW + H_BP;             //    2200
                    const uint16_t V_TOTAL = V_RES + V_FP + V_SYNCW + V_BP;             //    1125
                    const uint32_t VS_HIGH = V_SYNCW * H_TOTAL;                         //   11000
                    const uint32_t VS_LOW = ( V_RES + V_FP + V_BP ) * H_TOTAL;          // 2464000
                    const uint16_t HS_LOW = H_RES + H_FP + H_BP;                        //    2156
                    const uint32_t V2D = ( V_SYNCW + V_BP ) * H_TOTAL + H_SYNCW + H_BP; //   90392
                    const uint16_t DE_LOW = H_FP + H_SYNCW + H_BP;                      //     280

                    // variables
                    uint8_t highByte = 0;
                    uint8_t midByte = 0;
                    uint8_t lowByte = 0;

                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_VTX0, 0x04, 0x00, false, 0 ) );   // do not invert DE
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_VTX0, 0x08, 0x00, false, 0 ) );   // do not invert HS
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_VTX0, 0x10, 0x10, false, 0 ) );   // invert VS

                    getHighMidLow( VS_DLY, &highByte, &midByte, &lowByte );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX2,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX2 + 1, midByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX2 + 2, lowByte, false ) );

                    getHighMidLow( VS_HIGH, &highByte, &midByte, &lowByte );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX5,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX5 + 1, midByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX5 + 2, lowByte, false ) );

                    getHighMidLow( VS_LOW, &highByte, &midByte, &lowByte );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX8,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX8 + 1, midByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX8 + 2, lowByte, false ) );

                    getHighMidLow( HS_DLY, &highByte, &midByte, &lowByte );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX11,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX11 + 1, midByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX11 + 2, lowByte, false ) );

                    getHighLow( H_SYNCW, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX14,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX14 + 1, lowByte, false ) );

                    getHighLow( HS_LOW, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX16,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX16 + 1, lowByte, false ) );

                    getHighLow( V_TOTAL, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX18,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX18 + 1, lowByte, false ) );

                    getHighMidLow( V2D, &highByte, &midByte, &lowByte );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX20,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX20 + 1, midByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX20 + 2, lowByte, false ) );

                    getHighLow( H_RES, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX23,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX23 + 1, lowByte, false ) );

                    getHighLow( DE_LOW, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX25,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX25 + 1, lowByte, false ) );

                    getHighLow( V_RES, &highByte, &lowByte );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX27,     highByte, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX27 + 1, lowByte, false ) ); 

                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_VTX29, 0x03, 0x01, false, 0 ) );   // patgen_mode = checkerboard

                    // checker color A
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX31,     CHECKER_COLOR_A_B, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX32,     CHECKER_COLOR_A_G, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX33,     CHECKER_COLOR_A_R, false ) );

                    // checker color B
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX34,     CHECKER_COLOR_B_B, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX35,     CHECKER_COLOR_B_G, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX36,     CHECKER_COLOR_B_R, false ) );

                    // dimensions
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX37,     CHECKER_WIDTH_A, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX38,     CHECKER_WIDTH_B, false ) );
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX39,     CHECKER_HEIGHT,  false ) );

                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_REG7,      0x01, 0x00, false, 0 ) );   // par_vid_en = 0
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_VTX1,      0x0e, 0x0a, false, 0 ) );   // patgen_clk_src = 5
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_VIDEO_TX0, 0x08, 0x00, false, 0 ) );   // auto_bpp = 0
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_EXT11,     0x80, 0x00, false, 0 ) );   // tun_mode = 0
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_VTX0,      0xe3, 0xe3, false, 0 ) );   // Generate VS and HS and DE, free-running mode, do not touch inversion
                }
                break;

            case SEQUENCE_SER_MAX96717_PATTERN_OFF:
                {
                    const uint8_t SER_I2C_ADDRESS = platformInstance->getSerI2cSlaveAddress();
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX29, 0x00, false ) );    // patgen_mode = none
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_VTX0,  0x03, false ) );    // patgen reset
                }
                break;

            case SEQUENCE_CAM_AR0234_SER_MAX96717_DES_MAX96724:
                {
                    const uint8_t DES_I2C_ADDRESS = platformInstance->getDesI2cSlaveAddress();
                    const uint8_t SER_I2C_ADDRESS = platformInstance->getSerI2cSlaveAddress();

                    ///////////////////////////////////////////
                    // MAX96717 - setup
                    ///////////////////////////////////////////
                    // set VREG_PFDDIV = 1.1V
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_CMU2, 0x70, 0x10, false, 0 ) );

                    // set 2 data lanes
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_MIPI_RX1, 0x30, 0x10, false, 0 ) );

                    // enable tunneling mode
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, GmslAction::ACTION_TYPE_WRITE_WITH_MASK, MAX96717_REG_EXT11, 0x80, 0x80, false, 0 ) );

                    // MFP0 - camera reset, active low
                    // pull-down buffer, push-pull driver
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_GPIO_B_OF_GPIO0, 0xa0, false ) );
                    // drive output low (camera disabled)
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_GPIO_A_OF_GPIO0, 0x00, false ) );
                    // delay 10 ms
                    sequenceVec.push_back( GmslAction( 10000 ) );
                    // drive output high (camera enabled)
                    sequenceVec.push_back( GmslAction( SER_I2C_ADDRESS, MAX96717_REG_GPIO_A_OF_GPIO0, 0x10, false ) );
                    // delay 10 ms
                    sequenceVec.push_back( GmslAction( 10000 ) );


                    ///////////////////////////////////////////
                    // MAX96724 - setup
                    ///////////////////////////////////////////
                    // RESET_LINK=1
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_CTRL1, 0xf0, false ) );

                    // MIPI D-PHY Setup
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_MIPI_PHY0, 0x01, false ) );  // configure for 4x2 mode
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_MIPI_PHY4, 0x44, false ) );  // MIPI PHY 2 & 3: D0 -> D0, D1 -> D1

                    // MIPI PHY 2
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_MIPI_TX10_OF_MIPI_TX2, 0x50, false ) );  // 2 data lanes
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_MIPI_TX54_OF_MIPI_TX2, 0x09, false ) );  // Enable tunneling
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_MIPI_TX57_OF_MIPI_TX2, 0x20, false ) );  // Tunnel destination controller 2

                    // GMSL Link Configuration
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_REG6, 0xf4, false ) );   // enable link C

                    // RESET_LINK=0
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_CTRL1, 0x00, false ) );

                    // RESET_ONESHOT
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_CTRL1, 0x0f, false ) );

                    // disable remote control channels A,B,D and enable I2C communications to camera on link C
                    sequenceVec.push_back( GmslAction( DES_I2C_ADDRESS, MAX96724_REG_REG3, 0xef, false ) );


                    ///////////////////////////////////////////
                    // ONSEMI AR0234 - BEGIN CONFIG
                    // 2 data lanes, 1920x1080, raw10, 60 fps, I2C_address=0x10
                    //
                    // https://www.openncc.com/product-page/openncc-2mp-global-shutter-module-daughterboard
                    // https://7e77f981-dec1-42d7-ac0b-cbdf3b6462f6.usrfiles.com/ugd/7e77f9_f4fb2edac73f4707aad1b4500ae2d6f1.txt
                    ///////////////////////////////////////////
                    const uint8_t CAM_I2C_ADDRESS = 0x10;

                    sequenceVec.push_back( GmslAction( 200 ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x301a, 0x00d9, true ) );
                    sequenceVec.push_back( GmslAction( 2000 ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3f4c, 0x121f, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3f4e, 0x121f, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3f50, 0x0b81, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31e0, 0x0003, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31e0, 0x0003, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x30b0, 0x0028, true ) );
                    sequenceVec.push_back( GmslAction( 200 ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x302a, 0x0005, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x302c, 0x0001, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x302e, 0x0008, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3030, 0x0096, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3036, 0x000a, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3038, 0x0001, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x30b0, 0x0028, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x30b4, 0x0011, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3146, 0x0438, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31b0, 0x0082, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31b2, 0x005c, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31b4, 0x51c8, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31b6, 0x3257, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31b8, 0x904b, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31ba, 0x030b, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31bc, 0x8e09, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3354, 0x002b, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31d0, 0x0000, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31ae, 0x0204, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3002, 0x0008, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3004, 0x0008, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3006, 0x043f, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3008, 0x0787, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3064, 0x1802, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x300a, 0x04c4, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x300c, 0x0264, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x30a2, 0x0001, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x30a6, 0x0001, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3012, 0x0168, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3786, 0x0006, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x31ae, 0x0202, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3096, 0x0280, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3088, 0x81ba, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3086, 0x3d02, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3090, 0x043f, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3ed2, 0xfa86, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3180, 0x824f, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3ecc, 0x6d42, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3ecc, 0x0d42, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x30ba, 0x7622, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3102, 0x5000, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3060, 0x000d, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3ed2, 0xaa86, true ) );
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3eee, 0xa4aa, true ) );

                    // comment the next line for starting directly the video feed
                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x3070, 0x0003, true ) );

                    sequenceVec.push_back( GmslAction( CAM_I2C_ADDRESS, 0x301a, 0x225c, true ) );
                    ///////////////////////////////////////////
                    // ONSEMI AR0234 - END CONFIG
                    ///////////////////////////////////////////
                }
                break;

            case SEQUENCE_NONE:
            default:
                break;
        }
    }

    return sequenceVec;
}


//!************************************************************************
//! Create the map of sequences
//!
//! @returns nothing
//!************************************************************************
void GmslI2cCfg::createSequencesMap()
{
    mSequencesMap.insert( std::make_pair( SEQUENCE_DES_MAX96724_READ_REGISTER,
                                          "DES MAX96724 -> Read a register" ) );
    mSequencesMap.insert( std::make_pair( SEQUENCE_DES_MAX96724_READ_DEVICE_ID,
                                          "DES MAX96724 -> Read device ID" ) );
    mSequencesMap.insert( std::make_pair( SEQUENCE_DES_MAX96724_TOGGLE_LOCK_LED,
                                          "DES MAX96724 -> Toggle 5 times the LOCK LED" ) );
    mSequencesMap.insert( std::make_pair( SEQUENCE_DES_MAX96724_RESET,
                                          "DES MAX96724 -> Reset" ) );
    mSequencesMap.insert( std::make_pair( SEQUENCE_DES_MAX96724_SET_VIDEO_FMT_CODE_RGB888,
                                          "DES MAX96724 -> Set video format RGB888" ) );
    mSequencesMap.insert( std::make_pair( SEQUENCE_DES_MAX96724_SET_VIDEO_FMT_CODE_RAW10,
                                          "DES MAX96724 -> Set video format raw10 grbg" ) );

    mSequencesMap.insert( std::make_pair( SEQUENCE_DES_MAX96724_KERNEL_PATTERN_COLORBARS_ON,
                                          "DES MAX96724 -> Pattern colorbars ON (kernel function)" ) );
    mSequencesMap.insert( std::make_pair( SEQUENCE_DES_MAX96724_KERNEL_PATTERN_OFF,
                                          "DES MAX96724 -> Pattern OFF (kernel function)" ) );

    mSequencesMap.insert( std::make_pair( SEQUENCE_DES_MAX96724_USERSPACE_PATTERN_COLORBARS_ON,
                                          "DES MAX96724 -> Pattern colorbars ON (userspace)" ) );
    mSequencesMap.insert( std::make_pair( SEQUENCE_DES_MAX96724_USERSPACE_PATTERN_OFF,
                                          "DES MAX96724 -> Pattern OFF (userspace)" ) );

    mSequencesMap.insert( std::make_pair( SEQUENCE_SER_MAX96717_READ_REGISTER,
                                          "SER MAX96717 -> Read a register" ) );
    mSequencesMap.insert( std::make_pair( SEQUENCE_SER_MAX96717_READ_DEVICE_ID,
                                          "SER MAX96717 -> Read device ID" ) );
    mSequencesMap.insert( std::make_pair( SEQUENCE_SER_MAX96717_RESET,
                                          "SER MAX96717 -> Reset" ) );

    mSequencesMap.insert( std::make_pair( SEQUENCE_SER_MAX96717_PATTERN_CHECKERBOARD_ON,
                                          "SER MAX96717 -> Pattern checkerboard ON" ) );
    mSequencesMap.insert( std::make_pair( SEQUENCE_SER_MAX96717_PATTERN_OFF,
                                          "SER MAX96717 -> Pattern OFF" ) );

    mSequencesMap.insert( std::make_pair( SEQUENCE_CAM_AR0234_SER_MAX96717_DES_MAX96724,
                                          "AR0234 & SER MAX96717 & DES MAX96724 (tunneling)" ) );

    mSequencesMap.insert( std::make_pair( SEQUENCE_NONE,
                                          "Exit" ) );
}


//!************************************************************************
//! Extract the H and L bytes of a word
//! B1=H, B0=L
//!
//! @returns nothing
//!************************************************************************
void GmslI2cCfg::getHighLow
    (
    const uint16_t  aWord,          //!< word
    uint8_t*        aHighByte,      //!< high byte
    uint8_t*        aLowByte        //!< low byte
    ) const
{
    if( aHighByte && aLowByte )
    {
        *aLowByte = static_cast<uint8_t>( aWord & 0x00FF );
        *aHighByte = static_cast<uint8_t>( ( aWord >> 8 ) & 0x00FF );
    }
}


//!************************************************************************
//! Extract the H, M, and L bytes of a double word
//! B3 is not considered, B2=H, B1=M, B0=L
//!
//! @returns nothing
//!************************************************************************        
void GmslI2cCfg::getHighMidLow
    (
    const uint32_t  aDoubleWord,    //!< double word
    uint8_t*        aHighByte,      //!< high byte
    uint8_t*        aMidByte,       //!< mid byte
    uint8_t*        aLowByte        //!< low byte
    ) const
{
    if( aHighByte && aMidByte && aLowByte )
    {
        *aLowByte = static_cast<uint8_t>( aDoubleWord & 0x000000FF );
        *aMidByte = static_cast<uint8_t>( ( aDoubleWord >> 8 ) & 0x000000FF );
        *aHighByte = static_cast<uint8_t>( ( aDoubleWord >> 16 ) & 0x000000FF );
    }
}


//!************************************************************************
//! Read a 16-bit register address from console.
//! The address is not checked to belong to a particular register map.
//!
//! @returns A 16-bit register address entered by the user
//!************************************************************************
uint16_t GmslI2cCfg::getUserRegAddress() const
{
    std::string regStr;
    bool formatCorrect = false;

    do{
        std::cout << "\n Enter a 16-bit register address using the format 0xHHHH" ;
        std::cout << "\n Address = 0x";
        std::cin >> regStr;
        formatCorrect = ( 4 == regStr.size() && ( std::all_of( regStr.begin(), regStr.end(), ::isxdigit ) ) );
    }while( !formatCorrect );

    return std::stoul( regStr, nullptr, 16 );
}


//!************************************************************************
//! Get a character from the keyboard
//!
//! @returns The ASCII code of the character
//!************************************************************************
int GmslI2cCfg::getch()
{
    struct termios oldt;
    tcgetattr( STDIN_FILENO, &oldt );
    struct termios newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    int ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

    return ch;
}


//!************************************************************************
//! Waits for a key to be pressed
//!
//! @returns The read count
//!************************************************************************
int GmslI2cCfg::kbhit()
{
    static struct termios oldt;
    tcgetattr( STDIN_FILENO, &oldt );
    static struct termios newt = oldt;
    newt.c_lflag    &= ~( ICANON | ECHO );
    newt.c_iflag     = 0;           // input mode
    newt.c_oflag     = 0;           // output mode
    newt.c_cc[VMIN]  = 1;           // minimum time to wait
    newt.c_cc[VTIME] = 1;           // minimum characters to wait for
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    int cnt = 0;
    ioctl( 0, FIONREAD, &cnt );     // read count
    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 50000;
    select( STDIN_FILENO + 1, NULL, NULL, NULL, &tv ); // delay
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

    return cnt;
}


//!************************************************************************
//! Print a list of available options
//!
//! @returns nothing
//!************************************************************************
void GmslI2cCfg::printOptions() const
{
    std::cout << "\n ====================================" ;
    std::cout << "\n Available GMSL sequence options are:" ;
    std::cout << "\n ====================================" ;

    for( auto const& crtSequence : mSequencesMap )
    {
        std::cout << "\n " << crtSequence.first << " - " << crtSequence.second;
    }

    std::cout << "\n ====================================" ;
    std::fflush( stdout );
}
