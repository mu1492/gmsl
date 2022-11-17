/*
GmslSequence.cpp

This file contains the sources for GMSL I2C commands sequence.


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

#include "GmslSequence.h"

//!************************************************************************
//! Constructor
//!************************************************************************
GmslSequence::GmslSequence()
{
}


//!************************************************************************
//! Clear the GMSL sequence
//!
//! @returns: nothing
//!************************************************************************
void GmslSequence::clearSequence()
{
    mActionsVector.clear();
}


//!************************************************************************
//! Get the GMSL sequence vector
//!
//! @returns: the GMSL sequence vector
//!************************************************************************
std::vector<GmslAction> GmslSequence::getVector() const
{
    return mActionsVector;
}


//!************************************************************************
//! Insert an action in the sequence vector
//!
//! @returns: nothing
//!************************************************************************
void GmslSequence::insertAction
    (
    const GmslAction     aGmslAction      //!< GMSL action to be inserted
    )
{
    mActionsVector.push_back( aGmslAction );
}
