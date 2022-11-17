/*
Platform.h

This file contains the definitions for embedded Linux platforms using I2C.


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

#ifndef PLATFORM_H
#define PLATFORM_H

#include <cstdint>
#include <string>
#include <vector>

#include "GmslAction.h"

//************************************************************************
// Class for handling platform-dependent I2C writes/reads
//************************************************************************
class Platform
{
    //************************************************************************
    // data types and constants
    //************************************************************************
    public:
        typedef enum : uint8_t
        {
            DESERIALIZER_MAX96724,

            DESERIALIZER_COUNT,
            DESERIALIZER_NONE = DESERIALIZER_COUNT
        }Deserializer;

        typedef enum : uint8_t
        {
            SERIALIZER_MAX96717,

            SERIALIZER_COUNT,
            SERIALIZER_NONE = SERIALIZER_COUNT
        }Serializer;

        // Attention: this value must match perfectly with the kernel assignment
        static const uint8_t DES_I2C_BUS = 10;

        // the number and order of items must match the kernel module parameters
        typedef enum : uint8_t
        {
            // I2C commands
            KERNEL_PARAMS_CMD_NONE,
            KERNEL_PARAMS_CMD_WRITE,
            KERNEL_PARAMS_CMD_READ,
            // Kernel function calls
            KERNEL_PARAMS_CMD_START_PATTERN_COLORBARS,
            KERNEL_PARAMS_CMD_STOP_PATTERN,
            KERNEL_PARAMS_CMD_SET_VIDEO_FMT_CODE_RGB888,
            KERNEL_PARAMS_CMD_SET_VIDEO_FMT_CODE_RAW10
        }KernelParamsCmd;


    private:
        const std::string MAX96724_KERNEL_PARAMETERS_PATH  = "/sys/bus/i2c/drivers/max96724/module/parameters/";

        const std::string KERNEL_PARAM_I2C_BUS_NR         = "i2c_bus_nr";
        const std::string KERNEL_PARAM_I2C_CMD            = "i2c_cmd";
        const std::string KERNEL_PARAM_I2C_REG_ADDR       = "i2c_reg_addr";
        const std::string KERNEL_PARAM_I2C_SLAVE_ADDR     = "i2c_slave_addr";
        const std::string KERNEL_PARAM_I2C_VALUE          = "i2c_value";
        const std::string KERNEL_PARAM_I2C_VALUE_IS_WORD  = "i2c_value_is_word";

        const std::string SUDO_WRITE_STR = "sudo sh -c \"echo ";

        typedef struct
        {
            uint8_t     i2cBusNr;
            uint8_t     i2cCmd;
            uint16_t    i2cRegAddr;
            uint8_t     i2cSlaveAddr;
            uint16_t    i2cValue;
            bool        i2cValueIsWord;
        }DesKernelParams;


    //************************************************************************
    // functions
    //************************************************************************
    public:
        Platform();

        ~Platform();

        static Platform* getInstance();

        void delayAction
            (
            const GmslAction aAction            //!< action object
            ) const;

        uint8_t getDesI2cSlaveAddress() const;

        uint8_t getSerI2cSlaveAddress() const;

        void readAction
            (
            const GmslAction aAction,           //!< action object
            uint8_t*         aValue             //!< read value
            );

        void setDes
            (
            const Deserializer aDeserializer    //!< deserializer
            );

        void setDesKernelParamI2cCmd
            (
            const uint8_t   aI2cCmd             //!< I2C command
            );

        void setSer
            (
            const Serializer aSerializer        //!< serializer
            );

        void updateDesFromKernel();

        void writeAction
            (
            const GmslAction aAction            //!< action object
            );

        void writeWithMaskAction
            (
            const GmslAction aAction            //!< action object
            );


    private:
        uint8_t  getDesKernelParamI2cBusNr() const;
        uint8_t  getDesKernelParamI2cCmd() const;
        uint16_t getDesKernelParamI2cRegAddr() const;
        uint8_t  getDesKernelParamI2cSlaveAddr() const;
        uint16_t getDesKernelParamI2cValue() const;
        bool     getDesKernelParamI2cValueIsWord() const;

        std::string getPathDesKernelParamI2cBusNr() const;
        std::string getPathDesKernelParamI2cCmd() const;
        std::string getPathDesKernelParamI2cRegAddr() const;
        std::string getPathDesKernelParamI2cSlaveAddr() const;
        std::string getPathDesKernelParamI2cValue() const;
        std::string getPathDesKernelParamI2cValueIsWord() const;

        void setDesKernelParamI2cBusNr
            (
            const uint8_t   aI2cBusNr           //!< I2C bus nr
            );
        void setDesKernelParamI2cRegAddr
            (
            const uint16_t  aI2cRegAddr         //!< I2C register address
            );
        void setDesKernelParamI2cSlaveAddr
            (
            const uint8_t   aI2cSlaveAddr       //!< I2C 7-bit slave address
            );
        void setDesKernelParamI2cValue
            (
            const uint16_t  aI2cValue           //!< I2C value
            );
        void setDesKernelParamI2cValueIsWord
            (
            const bool      aI2cValueIsWord     //!< true if the I2C value is word
            );


        bool readReg
            (
            const uint16_t 	aRegisterAddress,   //!< register address (word)
            uint8_t* 		aValue              //!< byte to read
            ) const;

        void readRegDes
            (
            const uint16_t 	aRegisterAddress,   //!< register address (word)
            uint8_t* 		aValue              //!< byte to read
            );


        bool writeReg
            (
            const uint16_t  aRegisterAddress,       //!< register address (word)
            const uint8_t 	aValue              //!< byte to write
            ) const;

        void writeRegDes
            (
            const uint16_t  aRegisterAddress,       //!< register address (word)
            const uint8_t 	aValue              //!< byte to write
            );


        bool writeReg
            (
            const uint16_t  aRegisterAddress,       //!< register address (word)
            const uint16_t 	aValue              //!< word to write
            ) const;

        void writeRegDes
            (
            const uint16_t  aRegisterAddress,       //!< register address (word)
            const uint16_t 	aValue              //!< word to write
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        static Platform*    sInstance;          //!< singleton

        Deserializer        mDes;               //!< deserializer
        DesKernelParams     mDesKernelParams;   //!< structure with deserializer kernel parameters

        Serializer          mSer;               //!< serializer

        int                 mI2cBusChannel;     //!< I2C bus channel
        bool                mI2cIsOpen;         //!< if I2C bus could be open
};

#endif // PLATFORM_H

