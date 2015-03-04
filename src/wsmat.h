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

/**
 * \file
 */

#ifndef RF_WSMAT_H
#define RF_WSMAT_H

#include <QCoreApplication>
#include <qimage.h>
#include <vector>

#include "Workspace/DataExecution/DataObjects/datafactorytraits.h"
#include "Workspace/DataExecution/DataObjects/objectgroup.h"
#include "Workspace/DataExecution/DataObjects/derivedtobaseadaptor.h"

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "structurefrommotionplugin.h"


// Forward declarations so that our get / set functions work.

namespace RF
{
    class WSMatImpl;

    /**
     * \brief Put a one-line description of your datatype here
     *
     * Add a more detailed description of your datatype here
     * or remove these lines if the brief description above
     * is sufficient.
     */
    class RF_API WSMat : public CSIRO::DataExecution::ObjectGroup, public cv::Mat
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(RF::WSMat)

        WSMatImpl*  pImpl_;

    public:
        WSMat();
        WSMat(const WSMat& other);
        using CSIRO::DataExecution::ObjectGroup::size;
        //cv::Mat mat;
        
        virtual ~WSMat();
        
        // Clones our data type
        virtual WSMat* clone() const;

        // Optional, but handy operators
        bool operator==(const WSMat& rhs) const;
        WSMat& operator=(const WSMat& rhs);

        // Get / set functions for modifying data members in code
        QImage convertToQImage(void);
        
    };
}

DECLARE_WORKSPACE_DATA_FACTORY(RF::WSMat, RF_API)
DECLARE_WORKSPACE_DERIVEDTOBASEADAPTOR(RF::WSMat, CSIRO::DataExecution::ObjectGroup, RF_API)

//Other OpenCV datatypes
DECLARE_WORKSPACE_DATA_FACTORY(cv::KeyPoint, CV_EXPORTS_W_SIMPLE)
DECLARE_WORKSPACE_DATA_FACTORY(std::vector<cv::KeyPoint>, CV_EXPORTS_W_SIMPLE)
DECLARE_WORKSPACE_DATA_FACTORY(std::vector<cv::DMatch>, CV_EXPORTS_W_SIMPLE)

#endif

