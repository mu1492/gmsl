/*
GmslAction.cpp

This file contains the sources for GMSL I2C actions.


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

#include "GmslAction.h"


//!************************************************************************
//! Constructor
//!************************************************************************
GmslAction::GmslAction
    (
    uint8_t     aI2cAddress,        //!< 7-bit I2C device address
    ActionType  aAction,            //!< action type
    uint16_t    aRegAddress,        //!< register address
    uint8_t     aMask,              //!< mask (byte)
    uint16_t    aValue,             //!< value (byte or word)
    bool        aIsValueWord,       //!< true if writing a word, false for byte
    uint16_t    aDelayUs            //!< delay [us]
    )
    : mI2cAddr( aI2cAddress )
    , mActionType( aAction )
    , mRegAddr( aRegAddress )
    , mMask( aMask )
    , mValue( aValue )
    , mIsValueWord( aIsValueWord )
    , mDelayUs( aDelayUs )
{
}


//!************************************************************************
//! Constructor for delay
//!************************************************************************
GmslAction::GmslAction
    (
    uint16_t    aDelayUs            //!< delay [us]
    )
    : mI2cAddr( 0 )
    , mActionType( ACTION_TYPE_DELAY )
    , mRegAddr( 0 )
    , mMask( 0 )
    , mValue( 0 )
    , mIsValueWord( false )
    , mDelayUs( aDelayUs )
{
}


//!************************************************************************
//! Constructor for write
//!************************************************************************
GmslAction::GmslAction
    (
    uint8_t     aI2cAddress,        //!< 7-bit I2C device address
    uint16_t    aRegAddress,        //!< register address
    uint16_t    aValue,             //!< value (byte or word)
    bool        aIsValueWord        //!< true if writing a word, false for byte
    )
    : mI2cAddr( aI2cAddress )
    , mActionType( ACTION_TYPE_WRITE )
    , mRegAddr( aRegAddress )
    , mMask( 0 )
    , mValue( aValue )
    , mIsValueWord( aIsValueWord )
    , mDelayUs( 0 )
{
}
