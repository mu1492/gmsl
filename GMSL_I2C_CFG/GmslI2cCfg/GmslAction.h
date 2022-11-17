/*
GmslAction.h

This file contains the definitions for GMSL I2C actions.


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

#ifndef GMSLACTION_H
#define GMSLACTION_H

#include <cstdint>
#include <string>
#include <vector>

//************************************************************************
// Class for handling GMSL actions
//************************************************************************
class GmslAction
{
    //**********************************************************************
    // data types and constants
    //**********************************************************************
    public:
        typedef enum : uint8_t
        {
            ACTION_TYPE_INVALID,

            ACTION_TYPE_WRITE,
            ACTION_TYPE_WRITE_WITH_MASK,
            ACTION_TYPE_DELAY,
            ACTION_TYPE_READ
        }ActionType;


    //**********************************************************************
    // functions
    //**********************************************************************
    public:       
        GmslAction
            (
            uint8_t     aI2cAddress,            //!< 7-bit I2C device address
            ActionType  aAction,                //!< action type
            uint16_t    aRegAddress,            //!< register address
            uint8_t     aMask,                  //!< mask (byte)
            uint16_t    aValue,                 //!< value (byte or word)
            bool        aIsValueWord,           //!< true if writing a word, false for byte
            uint16_t    aDelayUs                //!< delay [us]
            );

        GmslAction
            (
            uint16_t    aDelayUs                //!< delay [us]
            );

        GmslAction
            (
            uint8_t     aI2cAddress,            //!< 7-bit I2C device address
            uint16_t    aRegAddress,            //!< register address
            uint16_t    aValue,                 //!< value (word or byte)
            bool        aIsValueWord            //!< true if writing a word, false for byte
            );


    //***********************************************************************
    // variables
    //***********************************************************************
    public:
        uint8_t                 mI2cAddr;       //!< 7-bit I2C device address

        ActionType              mActionType;    //!< action type

        uint16_t                mRegAddr;       //!< register address

        uint8_t                 mMask;          //!< mask (byte)
        uint16_t                mValue;         //!< value (byte or word)
        bool                    mIsValueWord;   //!< true if writing a word, false for byte

        uint16_t                mDelayUs;       //!< delay [us]
};

#endif // GMSLACTION_H
