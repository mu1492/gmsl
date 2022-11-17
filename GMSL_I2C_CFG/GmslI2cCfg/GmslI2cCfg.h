/*
GmslI2cCfg.h

This file contains the definitions for GMSL I2C configurator.


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

#ifndef GMSLI2CCFG_H
#define GMSLI2CCFG_H

#include <map>
#include <string>

#include "GmslSequence.h"

//************************************************************************
// Class for handling GMSL I2C configurations
//************************************************************************
class GmslI2cCfg
{
    //**********************************************************************
    // data types and constants
    //**********************************************************************
    private:
        typedef enum : uint8_t
        {
            SEQUENCE_DES_MAX96724_READ_REGISTER,
            SEQUENCE_DES_MAX96724_READ_DEVICE_ID,
            SEQUENCE_DES_MAX96724_TOGGLE_LOCK_LED,
            SEQUENCE_DES_MAX96724_RESET,
            SEQUENCE_DES_MAX96724_SET_VIDEO_FMT_CODE_RGB888,
            SEQUENCE_DES_MAX96724_SET_VIDEO_FMT_CODE_RAW10,

            SEQUENCE_DES_MAX96724_KERNEL_PATTERN_COLORBARS_ON,
            SEQUENCE_DES_MAX96724_KERNEL_PATTERN_OFF,

            SEQUENCE_DES_MAX96724_USERSPACE_PATTERN_COLORBARS_ON,
            SEQUENCE_DES_MAX96724_USERSPACE_PATTERN_OFF,

            SEQUENCE_SER_MAX96717_READ_REGISTER,
            SEQUENCE_SER_MAX96717_READ_DEVICE_ID,
            SEQUENCE_SER_MAX96717_RESET,

            SEQUENCE_SER_MAX96717_PATTERN_CHECKERBOARD_ON,
            SEQUENCE_SER_MAX96717_PATTERN_OFF,

            SEQUENCE_CAM_AR0234_SER_MAX96717_DES_MAX96724,

            // keep these last
            SEQUENCE_COUNT,
            SEQUENCE_NONE = SEQUENCE_COUNT
        }Sequence;


        typedef enum : uint16_t
        {
            MAX96717_REG_REG7               = 0x0007,
            MAX96717_REG_REG13              = 0x000d,
            MAX96717_REG_CTRL0              = 0x0010,
            MAX96717_REG_INTR0              = 0x0018,
            MAX96717_REG_VIDEO_TX0          = 0x0110,
            MAX96717_REG_VTX0               = 0x024e,
            MAX96717_REG_VTX1               = 0x024f,
            MAX96717_REG_VTX2               = 0x0250,
            MAX96717_REG_VTX5               = 0x0253,
            MAX96717_REG_VTX8               = 0x0256,
            MAX96717_REG_VTX11              = 0x0259,
            MAX96717_REG_VTX14              = 0x025c,
            MAX96717_REG_VTX16              = 0x025e,
            MAX96717_REG_VTX18              = 0x0260,
            MAX96717_REG_VTX20              = 0x0262,
            MAX96717_REG_VTX23              = 0x0265,
            MAX96717_REG_VTX25              = 0x0267,
            MAX96717_REG_VTX27              = 0x0269,
            MAX96717_REG_VTX29              = 0x026b,
            MAX96717_REG_VTX31              = 0x026d,
            MAX96717_REG_VTX32              = 0x026e,
            MAX96717_REG_VTX33              = 0x026f,
            MAX96717_REG_VTX34              = 0x0270,
            MAX96717_REG_VTX35              = 0x0271,
            MAX96717_REG_VTX36              = 0x0272,
            MAX96717_REG_VTX37              = 0x0273,
            MAX96717_REG_VTX38              = 0x0274,
            MAX96717_REG_VTX39              = 0x0275,
            MAX96717_REG_GPIO_A_OF_GPIO0    = 0x02be,
            MAX96717_REG_GPIO_B_OF_GPIO0    = 0x02bf,
            MAX96717_REG_CMU2               = 0x0302,
            MAX96717_REG_FRONTTOP_11        = 0x0313,
            MAX96717_REG_FRONTTOP_16        = 0x0318,
            MAX96717_REG_FRONTTOP_22        = 0x031e,
            MAX96717_REG_MIPI_RX1           = 0x0331,
            MAX96717_REG_EXT11              = 0x0383
        }Max96717_Reg;


        typedef enum : uint16_t
        {
            MAX96724_REG_REG3                   = 0x0003,
            MAX96724_REG_REG5                   = 0x0005,
            MAX96724_REG_REG6                   = 0x0006,
            MAX96724_REG_PCLK_FREQ              = 0x0009,
            MAX96724_REG_REG13                  = 0x000d,
            MAX96724_REG_PWR1                   = 0x0013,
            MAX96724_REG_CTRL1                  = 0x0018,
            MAX96724_REG_VIDEO_PIPE_SEL_1       = 0x00f1,
            MAX96724_REG_VIDEO_PIPE_EN          = 0x00f4,
            MAX96724_REG_VPRBS                  = 0x021c,
            MAX96724_REG_BACKTOP12_OF_BACKTOP   = 0x040b,
            MAX96724_REG_BACKTOP28_OF_BACKTOP   = 0x041b,
            MAX96724_REG_BACKTOP5_OF_BACKTOP_1  = 0x0424,
            MAX96724_REG_FSYNC_0                = 0x04a0,
            MAX96724_REG_PROFILE_MIPI_SEL       = 0x06e1,
            MAX96724_REG_MIPI_PHY0              = 0x08a0,
            MAX96724_REG_MIPI_PHY2              = 0x08a2,
            MAX96724_REG_MIPI_PHY4              = 0x08a4,
            MAX96724_REG_MIPI_TX51_OF_MIPI_TX2  = 0x09b3,
            MAX96724_REG_MIPI_TX54_OF_MIPI_TX2  = 0x09b6,
            MAX96724_REG_MIPI_TX57_OF_MIPI_TX2  = 0x09b9,
            MAX96724_REG_MIPI_TX10_OF_MIPI_TX2  = 0x098a,
            MAX96724_REG_PATGEN_0               = 0x1050,
            MAX96724_REG_PATGEN_1               = 0x1051,
            MAX96724_REG_VS_DLY_2               = 0x1052,
            MAX96724_REG_VS_HIGH_2              = 0x1055,
            MAX96724_REG_VS_LOW_2               = 0x1058,
            MAX96724_REG_V2H_2                  = 0x105b,
            MAX96724_REG_HS_HIGH_1              = 0x105e,
            MAX96724_REG_HS_LOW_1               = 0x1060,
            MAX96724_REG_HS_CNT_1               = 0x1062,
            MAX96724_REG_V2D_2                  = 0x1064,
            MAX96724_REG_DE_HIGH_1              = 0x1067,
            MAX96724_REG_DE_LOW_1               = 0x1069,
            MAX96724_REG_DE_CNT_1               = 0x106b,
            MAX96724_REG_GRAD_INCR              = 0x106d,
            MAX96724_REG_DP_ORSTB_CTL           = 0x1191
        }Max96724_Reg;


    //**********************************************************************
    // functions
    //**********************************************************************
    public:
        GmslI2cCfg();

    private:
        std::vector<GmslAction> createSequence
            (
            const Sequence aIndex           //!< sequence index
            );

        void createSequencesMap();

        void getHighLow
            (
            const uint16_t  aWord,          //!< word
            uint8_t*        aHighByte,      //!< high byte
            uint8_t*        aLowByte        //!< low byte
            ) const;

        void getHighMidLow
            (
            const uint32_t  aDoubleWord,    //!< double word
            uint8_t*        aHighByte,      //!< high byte
            uint8_t*        aMidByte,       //!< mid byte
            uint8_t*        aLowByte        //!< low byte
            ) const;

        uint16_t getUserRegAddress() const;

        int getch();

        int kbhit();

        void printOptions() const;


    //***********************************************************************
    // variables
    //***********************************************************************
    private:
        GmslSequence                mSequence;          //!< sequence object
        std::map<int, std::string>  mSequencesMap;      //!< map of sequences
        uint8_t                     mSequenceIndex;     //!< sequence index
};

#endif // GMSLI2CCFG_H
