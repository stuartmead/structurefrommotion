/*
Copyright (c) 2015, Stuart Mead - Risk Frontiers, Macquarie University
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
      
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
      
    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software without
      specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  For further information, contact:
    Stuart Mead
    Risk Frontiers
    Dept. of Environmental Sciences
    Macquarie University
    North Ryde NSW 2109
*/

#include <cassert>
#include <iostream>

#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/DataObjects/typeddatafactory.h"


#include "structurefrommotionplugin.h"
#include "opencv2/features2d/features2d.hpp"
#include "wsmat.h"


namespace RF
{
    using namespace RF;
    using namespace CSIRO::DataExecution;

    /**
     * \internal
     */
    class WSMatImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(RF::WSMatImpl)

        void setObjects();

    public:
        WSMat&  owner_;

        // Data objects


        WSMatImpl(WSMat& owner);
        WSMatImpl(WSMat& owner, const WSMatImpl& other);
    };


    /**
     *
     */
    WSMatImpl::WSMatImpl(WSMat& owner) :
        owner_(owner)
    {
        setObjects();
    }


    /**
     *
     */
    WSMatImpl::WSMatImpl(WSMat& owner, const WSMatImpl& other) :
        owner_(owner)
    {
        setObjects();
    }


    /**
     *
     */
    void  WSMatImpl::setObjects()
    {

    }


    //==========================//


    /**
     *
     */
    WSMat::WSMat() :
        CSIRO::DataExecution::ObjectGroup()
    {
        pImpl_ = new WSMatImpl(*this);
    }


    /**
     *
     */
    WSMat::WSMat(const WSMat& other) :
        CSIRO::DataExecution::ObjectGroup()
    {
        pImpl_ = new WSMatImpl(*this, *other.pImpl_);
    }


    /**
     *
     */
    WSMat::~WSMat()
    {
        delete pImpl_;
    }


    /**
     * Cloning
     */
    WSMat* WSMat::clone() const
    {
        return new WSMat(*this);
    }


    /**
     * Comparison
     */
    bool WSMat::operator==(const WSMat& rhs) const
    {
        if (&rhs == this)
            return true;


        return true;
    }


    /**
     * Assignment
     */
    WSMat& WSMat::operator=(const WSMat& rhs)
    {
        // Check for self assignment
        if (&rhs == this)
            return *this;

        // Clear the current contents; we're about to delete the impl, so we
        // don't want any dangling pointers to our dataobjects.
        clear();

        WSMatImpl* impl = pImpl_;
        pImpl_ = new WSMatImpl(*this, *rhs.pImpl_);
        delete impl;
        impl = 0;

        return *this;
    }

    QImage WSMat::convertToQImage(void)
    {
        QImage qtmp((uchar*) data, cols, rows, step,QImage::Format_RGB888);
        QImage qtmp2(qtmp);
        qtmp2.detach();
        return qtmp2;
    }
    
}


DEFINE_WORKSPACE_DATA_FACTORY(RF::WSMat, RF::StructureFromMotionPlugin::getInstance())
DEFINE_WORKSPACE_DERIVEDTOBASEADAPTOR(RF::WSMat, CSIRO::DataExecution::ObjectGroup, RF::StructureFromMotionPlugin::getInstance())

DEFINE_WORKSPACE_DATA_FACTORY(cv::KeyPoint, RF::StructureFromMotionPlugin::getInstance())
DEFINE_WORKSPACE_DATA_FACTORY(std::vector<cv::KeyPoint>, RF::StructureFromMotionPlugin::getInstance())
DEFINE_WORKSPACE_DATA_FACTORY(std::vector<cv::DMatch>, RF::StructureFromMotionPlugin::getInstance())
