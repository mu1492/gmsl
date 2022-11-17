/*
Platform.cpp

This file contains the sources for embedded Linux platforms using I2C.


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

#include "Platform.h"

#include <cstring>
#include <fstream>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>


Platform* Platform::sInstance = nullptr;

//!************************************************************************
//! Constructor
//!************************************************************************
Platform::Platform()
    : mDes( DESERIALIZER_NONE )
    , mSer( SERIALIZER_NONE )
    , mI2cBusChannel( 0 )
    , mI2cIsOpen( false )
{
    memset( &mDesKernelParams, 0, sizeof( mDesKernelParams ) );

    std::string i2cPath = "/dev/i2c-" + std::to_string( DES_I2C_BUS );    
    mI2cBusChannel = open( i2cPath.c_str(), O_RDWR );

    if( mI2cBusChannel >= 0 )
    {
        mI2cIsOpen = true;
    }
    else
    {
        std::cout << "\n Could not open I2C bus " << i2cPath << "\n";
    }
}


//!************************************************************************
//! Destructor
//!************************************************************************
Platform::~Platform()
{
    if( mI2cIsOpen )
    {
        close( mI2cBusChannel );
    }
}


//!************************************************************************
//! Singleton
//!
//! @returns the instance of the object
//!************************************************************************
Platform* Platform::getInstance()
{
    if( !sInstance )
    {
        sInstance = new Platform;
    }

    return sInstance;
}


//!************************************************************************
//! Perform a GMSL delay action using I2C
//!
//! @returns: nothing
//!************************************************************************
void Platform::delayAction
    (
    const GmslAction aAction             //!< action object
    ) const
{
    usleep( aAction.mDelayUs );
}


//!************************************************************************
//! Get the deserializer I2C 7-bit slave address
//!
//! @returns: the 7-bit I2C slave address of the deserializer
//!************************************************************************
uint8_t Platform::getDesI2cSlaveAddress() const
{
    uint8_t retVal = 0;

    switch( mDes )
    {
        case DESERIALIZER_MAX96724:
            retVal = 0x4F;
            break;

        default:
            break;
    }

    return retVal >> 1;
}


//!************************************************************************
//! Get the serializer I2C 7-bit slave address
//!
//! @returns: the 7-bit I2C slave address of the serializer
//!************************************************************************
uint8_t Platform::getSerI2cSlaveAddress() const
{
    uint8_t retVal = 0;

    switch( mSer )
    {
        case SERIALIZER_MAX96717:
            retVal = 0x80;
            break;

        default:
            break;
    }

    return retVal >> 1;
}


//!************************************************************************
//! Get the i2c_bus_nr kernel parameter for deserializer
//!
//! @returns: the value of i2c_bus_nr
//!************************************************************************
uint8_t Platform::getDesKernelParamI2cBusNr() const
{
    uint8_t retVal = 0;
    FILE* fp = fopen( getPathDesKernelParamI2cBusNr().c_str() , "r" );

    if( fp )
    {
        fscanf( fp, "%hu", &retVal );
        fclose( fp );
    }

    return retVal;
}


//!************************************************************************
//! Get the i2c_cmd kernel parameter for deserializer
//!
//! @returns: the value of i2c_cmd
//!************************************************************************
uint8_t Platform::getDesKernelParamI2cCmd() const
{
    uint8_t retVal = 0;
    FILE* fp = fopen( getPathDesKernelParamI2cCmd().c_str() , "r" );

    if( fp )
    {
        fscanf( fp, "%hu", &retVal );
        fclose( fp );
    }

    return retVal;
}


//!************************************************************************
//! Get the i2c_reg_addr kernel parameter for deserializer
//!
//! @returns: the value of i2c_reg_addr
//!************************************************************************
uint16_t Platform::getDesKernelParamI2cRegAddr() const
{
    uint16_t retVal = 0;
    FILE* fp = fopen( getPathDesKernelParamI2cRegAddr().c_str() , "r" );

    if( fp )
    {
        fscanf( fp, "%hu", &retVal );
        fclose( fp );
    }

    return retVal;
}


//!************************************************************************
//! Get the i2c_slave_addr kernel parameter for deserializer
//!
//! @returns: the value of i2c_slave_addr
//!************************************************************************
uint8_t Platform::getDesKernelParamI2cSlaveAddr() const
{
    uint8_t retVal = 0;
    FILE* fp = fopen( getPathDesKernelParamI2cSlaveAddr().c_str() , "r" );

    if( fp )
    {
        fscanf( fp, "%hu", &retVal );
        fclose( fp );
    }

    return retVal;
}


//!************************************************************************
//! Get the i2c_value kernel parameter for deserializer
//!
//! @returns: the value of i2c_value
//!************************************************************************
uint16_t Platform::getDesKernelParamI2cValue() const
{
    uint16_t retVal = 0;
    FILE* fp = fopen( getPathDesKernelParamI2cValue().c_str() , "r" );

    if( fp )
    {
        fscanf( fp, "%hu", &retVal );
        fclose( fp );
    }

    return retVal;
}


//!************************************************************************
//! Get the i2c_value_is_word kernel parameter for deserializer
//!
//! @returns: true if i2c_value_is_word is 'y' or 'Y', false otherwise
//!************************************************************************
bool Platform::getDesKernelParamI2cValueIsWord() const
{
    bool retVal = false;
    FILE* fp = fopen( getPathDesKernelParamI2cValueIsWord().c_str() , "r" );

    if( fp )
    {
        char isWord = 0;
        fscanf( fp, "%c", &isWord );
        retVal = ( 'y'  == isWord || 'Y' == isWord );

        fclose( fp );
    }

    return retVal;
}


//!************************************************************************
//! Get the path of the i2c_bus_nr kernel parameter for deserializer
//!
//! @returns: the string containing the path
//!************************************************************************
std::string Platform::getPathDesKernelParamI2cBusNr() const
{
    std::string path;

    switch( mDes )
    {
        case DESERIALIZER_MAX96724:
            path = MAX96724_KERNEL_PARAMETERS_PATH + KERNEL_PARAM_I2C_BUS_NR;

        default:
            break;
    }

    return path;
}


//!************************************************************************
//! Get the path of the i2c_cmd kernel parameter for deserializer
//!
//! @returns: the string containing the path
//!************************************************************************
std::string Platform::getPathDesKernelParamI2cCmd() const
{
    std::string path;

    switch( mDes )
    {
        case DESERIALIZER_MAX96724:
            path = MAX96724_KERNEL_PARAMETERS_PATH + KERNEL_PARAM_I2C_CMD;

        default:
            break;
    }

    return path;
}


//!************************************************************************
//! Get the path of the i2c_reg_addr kernel parameter for deserializer
//!
//! @returns: the string containing the path
//!************************************************************************
std::string Platform::getPathDesKernelParamI2cRegAddr() const
{
    std::string path;

    switch( mDes )
    {
        case DESERIALIZER_MAX96724:
            path = MAX96724_KERNEL_PARAMETERS_PATH + KERNEL_PARAM_I2C_REG_ADDR;

        default:
            break;
    }

    return path;
}


//!************************************************************************
//! Get the path of the i2c_slave_addr kernel parameter for deserializer
//!
//! @returns: the string containing the path
//!************************************************************************
std::string Platform::getPathDesKernelParamI2cSlaveAddr() const
{
    std::string path;

    switch( mDes )
    {
        case DESERIALIZER_MAX96724:
            path = MAX96724_KERNEL_PARAMETERS_PATH + KERNEL_PARAM_I2C_SLAVE_ADDR;

        default:
            break;
    }

    return path;
}


//!************************************************************************
//! Get the path of the i2c_value kernel parameter for deserializer
//!
//! @returns: the string containing the path
//!************************************************************************
std::string Platform::getPathDesKernelParamI2cValue() const
{
    std::string path;

    switch( mDes )
    {
        case DESERIALIZER_MAX96724:
            path = MAX96724_KERNEL_PARAMETERS_PATH + KERNEL_PARAM_I2C_VALUE;

        default:
            break;
    }

    return path;
}


//!************************************************************************
//! Get the path of the i2c_value_is_word kernel parameter for deserializer
//!
//! @returns: the string containing the path
//!************************************************************************
std::string Platform::getPathDesKernelParamI2cValueIsWord() const
{
    std::string path;

    switch( mDes )
    {
        case DESERIALIZER_MAX96724:
            path = MAX96724_KERNEL_PARAMETERS_PATH + KERNEL_PARAM_I2C_VALUE_IS_WORD;

        default:
            break;
    }

    return path;
}


//!************************************************************************
//! Perform a read action using I2C
//!
//! @returns: nothing
//!************************************************************************
void Platform::readAction
    (
    const GmslAction aAction,           //!< action object
    uint8_t*         aValue             //!< read value
    )
{
    if( aValue )
    {
        *aValue = 0;

        if( getDesI2cSlaveAddress() == aAction.mI2cAddr ) // deserializer
        {
            // change mI2cAddr only if different from what kernel knows
            if( mDesKernelParams.i2cSlaveAddr != aAction.mI2cAddr )
            {
                setDesKernelParamI2cSlaveAddr( aAction.mI2cAddr );
            }

            // read from 16-bit address registers
            readRegDes( aAction.mRegAddr, aValue );
        }
        else // serializer(s) or camera(s)
        {
            if( ioctl( mI2cBusChannel, I2C_SLAVE, aAction.mI2cAddr ) >= 0 )
            {
                // read from 16-bit address registers
                readReg( aAction.mRegAddr, aValue );
            }
            else
            {
                std::cout << "\n Device at address 0x" << std::hex << static_cast<int>( aAction.mI2cAddr ) << " does not respond on I2C\n";
                std::cout << std::dec;
            }
        }
    }
}


//!************************************************************************
//! Read one byte from a register using 16-bit address.
//! Applies to other devices than deserializer.
//!
//! @returns: true if the value of the register can be read
//!************************************************************************
bool Platform::readReg
    (
    const uint16_t 	aRegisterAddress,   //!< register address
    uint8_t* 	  	aValue              //!< byte to read
    ) const
{
    bool couldRead = false;
    const int RX_LEN = 2;
    uint8_t rxBuffer[RX_LEN] = { 0 };

    rxBuffer[0] = static_cast<uint8_t>( ( aRegisterAddress >> 8 ) & 0x00ff );
    rxBuffer[1] = static_cast<uint8_t>( aRegisterAddress & 0x00ff );
    write( mI2cBusChannel, rxBuffer, sizeof( rxBuffer ) );

    if( RX_LEN == read( mI2cBusChannel, rxBuffer, sizeof( rxBuffer ) ) )
    {
        *aValue = rxBuffer[0];
        couldRead = true ;
    }
    else
    {
        std::cout << "\n Read from register address 0x" << std::hex << static_cast<int>( aRegisterAddress ) << " failed on I2C\n";
        std::cout << std::dec;
    }

    return couldRead;
}

//!************************************************************************
//! Read one byte from a register using 16-bit address.
//! Applies to deserializer.
//!
//! @returns: true if the value of the register can be read
//!************************************************************************
void Platform::readRegDes
    (
    const uint16_t 	aRegisterAddress,   //!< register address
    uint8_t* 	  	aValue              //!< byte to read
    )
{
    setDesKernelParamI2cRegAddr( aRegisterAddress );
    setDesKernelParamI2cValueIsWord( false );
    setDesKernelParamI2cCmd( KERNEL_PARAMS_CMD_READ );

    *aValue = static_cast<uint8_t>( getDesKernelParamI2cValue() );
}


//!************************************************************************
//! Set the deserializer type
//!
//! @returns: nothing
//!************************************************************************
void Platform::setDes
    (
    const Deserializer aDeserializer        //!< deserializer
    )
{
    if( aDeserializer < DESERIALIZER_COUNT )
    {
        mDes = aDeserializer;
    }

    mDesKernelParams.i2cBusNr = DES_I2C_BUS;
    setDesKernelParamI2cBusNr( mDesKernelParams.i2cBusNr );
}


//!************************************************************************
//! Set the i2c_bus_nr kernel parameter for deserializer
//!
//! @returns: nothing
//!************************************************************************
void Platform::setDesKernelParamI2cBusNr
    (
    const uint8_t   aI2cBusNr           //!< I2C bus nr
    )
{
    mDesKernelParams.i2cBusNr = aI2cBusNr;

    std::string cmdString = SUDO_WRITE_STR;
    cmdString += std::to_string( aI2cBusNr );
    cmdString += " > ";
    cmdString += getPathDesKernelParamI2cBusNr();
    cmdString += "\"";

    system( cmdString.c_str() );
}


//!************************************************************************
//! Set the i2c_cmd kernel parameter for deserializer
//!
//! @returns: nothing
//!************************************************************************
void Platform::setDesKernelParamI2cCmd
    (
    const uint8_t   aI2cCmd             //!< I2C command
    )
{
    mDesKernelParams.i2cCmd = aI2cCmd;

    std::string cmdString = SUDO_WRITE_STR;
    cmdString += std::to_string( aI2cCmd );
    cmdString += " > ";
    cmdString += getPathDesKernelParamI2cCmd();
    cmdString += "\"";

    system( cmdString.c_str() );
}


//!************************************************************************
//! Set the i2c_reg_addr kernel parameter for deserializer
//!
//! @returns: nothing
//!************************************************************************
void Platform::setDesKernelParamI2cRegAddr
    (
    const uint16_t  aI2cRegAddr         //!< I2C register address
    )
{
    mDesKernelParams.i2cRegAddr = aI2cRegAddr;

    std::string cmdString = SUDO_WRITE_STR;
    cmdString += std::to_string( aI2cRegAddr );
    cmdString += " > ";
    cmdString += getPathDesKernelParamI2cRegAddr();
    cmdString += "\"";

    system( cmdString.c_str() );
}


//!************************************************************************
//! Set the i2c_slave_addr kernel parameter for deserializer
//!
//! @returns: nothing
//!************************************************************************
void Platform::setDesKernelParamI2cSlaveAddr
    (
    const uint8_t   aI2cSlaveAddr       //!< I2C 7-bit slave address
    )
{
    mDesKernelParams.i2cSlaveAddr = aI2cSlaveAddr;

    std::string cmdString = SUDO_WRITE_STR;
    cmdString += std::to_string( aI2cSlaveAddr );
    cmdString += " > ";
    cmdString += getPathDesKernelParamI2cSlaveAddr();
    cmdString += "\"";

    system( cmdString.c_str() );
}


//!************************************************************************
//! Set the i2c_value kernel parameter for deserializer
//!
//! @returns: nothing
//!************************************************************************
void Platform::setDesKernelParamI2cValue
    (
    const uint16_t  aI2cValue           //!< I2C value
    )
{
    mDesKernelParams.i2cValue = aI2cValue;

    std::string cmdString = SUDO_WRITE_STR;
    cmdString += std::to_string( aI2cValue );
    cmdString += " > ";
    cmdString += getPathDesKernelParamI2cValue();
    cmdString += "\"";

    system( cmdString.c_str() );
}


//!************************************************************************
//! Set the i2c_value_is_word kernel parameter for deserializer
//!
//! @returns: nothing
//!************************************************************************
void Platform::setDesKernelParamI2cValueIsWord
    (
    const bool      aI2cValueIsWord     //!< true if the I2C value is word
    )
{
    mDesKernelParams.i2cValueIsWord = aI2cValueIsWord;

    std::string cmdString = SUDO_WRITE_STR;
    cmdString += aI2cValueIsWord ? "y" : "n";
    cmdString += " > ";
    cmdString += getPathDesKernelParamI2cValueIsWord();
    cmdString += "\"";

    system( cmdString.c_str() );
}


//!************************************************************************
//! Set the serializer type
//!
//! @returns: nothing
//!************************************************************************
void Platform::setSer
    (
    const Serializer aSerializer        //!< serializer
    )
{
    if( aSerializer < SERIALIZER_COUNT )
    {
        mSer = aSerializer;
    }
}


//!************************************************************************
//! Update the deserializer structure
//!
//! @returns: nothing
//!************************************************************************
void Platform::updateDesFromKernel()
{
    mDesKernelParams.i2cBusNr = getDesKernelParamI2cBusNr();
    mDesKernelParams.i2cCmd = getDesKernelParamI2cCmd();
    mDesKernelParams.i2cRegAddr = getDesKernelParamI2cRegAddr();
    mDesKernelParams.i2cSlaveAddr = getDesKernelParamI2cSlaveAddr();
    mDesKernelParams.i2cValue = getDesKernelParamI2cValue();
    mDesKernelParams.i2cValueIsWord = getDesKernelParamI2cValueIsWord();
}


//!************************************************************************
//! Perform a write action using I2C
//!
//! @returns: nothing
//!************************************************************************
void Platform::writeAction
    (
    const GmslAction aAction         //!< action to write
    )
{
    if( getDesI2cSlaveAddress() == aAction.mI2cAddr ) // deserializer
    {
        // change mI2cAddr only if different from what kernel knows
        if( mDesKernelParams.i2cSlaveAddr != aAction.mI2cAddr )
        {
            setDesKernelParamI2cSlaveAddr( aAction.mI2cAddr );
        }

        if( !aAction.mIsValueWord )
        {
            writeRegDes( aAction.mRegAddr, static_cast<uint8_t>( aAction.mValue & 0x00FF ) );
        }
        else
        {
            writeRegDes( aAction.mRegAddr, aAction.mValue );
        }
    }
    else // serializer(s) or camera(s)
    {
        if( ioctl( mI2cBusChannel, I2C_SLAVE, aAction.mI2cAddr ) >= 0 )
        {
            if( !aAction.mIsValueWord )
            {
                writeReg( aAction.mRegAddr, static_cast<uint8_t>( aAction.mValue & 0x00FF ) );
            }
            else
            {
                writeReg( aAction.mRegAddr, aAction.mValue );
            }
        }
        else
        {
            std::cout << "\n Device at address 0x" << std::hex << static_cast<int>( aAction.mI2cAddr ) << " does not respond on I2C\n";
            std::cout << std::dec;
        }
    }
}


//!************************************************************************
//! Perform a write with mask action using I2C
//!
//! @returns: nothing
//!************************************************************************
void Platform::writeWithMaskAction
    (
    const GmslAction aAction         //!< action to write
    )
{
    if( getDesI2cSlaveAddress() == aAction.mI2cAddr ) // deserializer
    {
        // change mI2cAddr only if different from what kernel knows
        if( mDesKernelParams.i2cSlaveAddr != aAction.mI2cAddr )
        {
            setDesKernelParamI2cSlaveAddr( aAction.mI2cAddr );
        }

        if( !aAction.mIsValueWord )
        {
            uint8_t crtValue = 0;
            readRegDes( aAction.mRegAddr, &crtValue );
            uint8_t notMask = static_cast<uint8_t>( ~aAction.mMask );
            uint8_t maskedCrtValue = notMask & crtValue;
            uint8_t focusValue = ( aAction.mValue & 0x00FF ) & aAction.mMask;
            uint8_t valueToWrite = focusValue | maskedCrtValue;
            writeRegDes( aAction.mRegAddr, valueToWrite );
        }
    }
    else // serializer(s) or camera(s)
    {
        if( !aAction.mIsValueWord )
        {
            if( ioctl( mI2cBusChannel, I2C_SLAVE, aAction.mI2cAddr ) >= 0 )
            {
                uint8_t crtValue = 0;
                readReg( aAction.mRegAddr, &crtValue );
                uint8_t notMask = static_cast<uint8_t>( ~aAction.mMask );
                uint8_t maskedCrtValue = notMask & crtValue;
                uint8_t focusValue = ( aAction.mValue & 0x00FF ) & aAction.mMask;
                uint8_t valueToWrite = focusValue | maskedCrtValue;
                writeReg( aAction.mRegAddr, valueToWrite );
            }
            else
            {
                std::cout << "\n Device at address 0x" << std::hex << static_cast<int>( aAction.mI2cAddr ) << " does not respond on I2C\n";
                std::cout << std::dec;
            }
        }
        else
        {
            std::cout << "\n Write with mask for 16-bit values is not implemented.\n";
        }
    }
}


//!************************************************************************
//! Write one byte to a register, address is word
//! Applies to other devices than deserializer.
//!
//! @returns: true if the value could be written to the register
//!************************************************************************
bool Platform::writeReg
    (
    const uint16_t aRegisterAddress,    //!< register address
    const uint8_t  aValue               //!< byte to write
    ) const
{
    bool couldWrite = false;
    const int TX_LEN = 3;
    uint8_t txBuffer[TX_LEN] = { 0 };

    txBuffer[0] = static_cast<uint8_t>( ( aRegisterAddress >> 8 ) & 0x00FF );
    txBuffer[1] = static_cast<uint8_t>( aRegisterAddress & 0x00FF );
    txBuffer[2] = aValue;

    if( TX_LEN == write( mI2cBusChannel, txBuffer, sizeof( txBuffer ) ) )
    {
        couldWrite = true;
    }
    else
    {
        std::cout << "\n Write at register address 0x" << std::hex << static_cast<int>( aRegisterAddress ) << " failed on I2C\n";
        std::cout << std::dec;
    }

    return couldWrite;
}

//!************************************************************************
//! Write one byte to a register, address is word
//! Applies to deserializer.
//!
//! @returns: true if the value could be written to the register
//!************************************************************************
void Platform::writeRegDes
    (
    const uint16_t aRegisterAddress,    //!< register address
    const uint8_t  aValue               //!< byte to write
    )
{
    setDesKernelParamI2cRegAddr( aRegisterAddress );
    setDesKernelParamI2cValueIsWord( false );
    setDesKernelParamI2cValue( aValue );
    setDesKernelParamI2cCmd( KERNEL_PARAMS_CMD_WRITE );
}


//!************************************************************************
//! Write one word to a register, address is word
//! Applies to other devices than deserializer.
//!
//! @returns: true if the value could be written to the register
//!************************************************************************
bool Platform::writeReg
    (
    const uint16_t aRegisterAddress,    //!< register address
    const uint16_t aValue               //!< word to write
    ) const
{
    bool couldWrite = false;
    const int TX_LEN = 4;
    uint8_t txBuffer[TX_LEN] = { 0 };

    txBuffer[0] = static_cast<uint8_t>( ( aRegisterAddress >> 8 ) & 0x00FF );
    txBuffer[1] = static_cast<uint8_t>( aRegisterAddress & 0x00FF );
    txBuffer[2] = static_cast<uint8_t>( ( aValue >> 8 ) & 0x00FF );
    txBuffer[3] = static_cast<uint8_t>( aValue & 0x00FF );

    if( TX_LEN == write( mI2cBusChannel, txBuffer, sizeof( txBuffer ) ) )
    {
        couldWrite = true;
    }
    else
    {
        std::cout << "\n Write at register address 0x" << std::hex << static_cast<int>( aRegisterAddress ) << " failed on I2C\n";
        std::cout << std::dec;
    }

    return couldWrite;
}

//!************************************************************************
//! Write one word to a register, address is word
//! Applies to deserializer.
//!
//! @returns: true if the value could be written to the register
//!************************************************************************
void Platform::writeRegDes
    (
    const uint16_t aRegisterAddress,    //!< register address
    const uint16_t aValue               //!< word to write
    )
{
    setDesKernelParamI2cRegAddr( aRegisterAddress );
    setDesKernelParamI2cValueIsWord( true );
    setDesKernelParamI2cValue( aValue );
    setDesKernelParamI2cCmd( KERNEL_PARAMS_CMD_WRITE );
}
