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
#include <fstream>

#include <QImage>
#include <qcolor.h>
#include <QString>

#include "Workspace/Application/LanguageUtils/streamqstring.h"
#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputscalar.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/output.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"

#include "structurefrommotionplugin.h"
#include "briskdetection.h"
#include "wsmat.h"


namespace RF
{
        /**
         * \internal
         */
    using namespace cv;
    class BriskDetectionImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(RF::BriskDetectionImpl)

    public:
        BriskDetection&  op_;

        // Data objects
        CSIRO::DataExecution::TypedObject< QString >  dataImage_;
        CSIRO::DataExecution::TypedObject< int >      dataThresh_;
        CSIRO::DataExecution::TypedObject< int >      dataOctave_;
        CSIRO::DataExecution::TypedObject< double >   dataPattern_;
        CSIRO::DataExecution::TypedObject< bool >     dataWriteSIFT_;
        CSIRO::DataExecution::TypedObject< bool >     data128bit_;
        CSIRO::DataExecution::TypedObject<std::vector<cv::KeyPoint> > dataKeypoint_;
        CSIRO::DataExecution::TypedObject<WSMat>      dataDescriptor_;


        // Inputs and outputs
        CSIRO::DataExecution::InputScalar inputImage_;
        CSIRO::DataExecution::InputScalar inputThresh_;
        CSIRO::DataExecution::InputScalar inputOctave_;
        CSIRO::DataExecution::InputScalar inputPattern_;
        CSIRO::DataExecution::InputScalar inputWriteSIFT_;
        CSIRO::DataExecution::InputScalar input128bit_;
        CSIRO::DataExecution::Output      outputKeypoint_;
        CSIRO::DataExecution::Output      outputDescriptor_;


        BriskDetectionImpl(BriskDetection& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    BriskDetectionImpl::BriskDetectionImpl(BriskDetection& op) :
        op_(op),
        dataImage_(),
        dataThresh_(30),
        dataOctave_(3),
        dataPattern_(1.0),
        dataWriteSIFT_(true),
        data128bit_(true),
        dataKeypoint_(),
        dataDescriptor_(),
        inputImage_("Image", dataImage_, op_),
        inputThresh_("FAST detection threshold score", dataThresh_, op_),
        inputOctave_("Number of detection octaves", dataOctave_, op_),
        inputPattern_("Pattern scale", dataPattern_, op_),
        inputWriteSIFT_("Write out SIFT files for VSFM", dataWriteSIFT_, op_),
        input128bit_("Use 128 bit descriptors", data128bit_, op_),
        outputKeypoint_("Keypoints", dataKeypoint_, op_),
        outputDescriptor_("Keypoint Descriptors", dataDescriptor_, op_)
    {
        inputThresh_.setDescription("FAST/AGAST detection threshold score.");
        inputOctave_.setDescription("Detection octaves. Use 0 to do single scale.");
        inputPattern_.setDescription("Apply this scale to the pattern used for sampling the neighbourhood of a keypoint.");
        inputWriteSIFT_.setDescription("Writes out a .sift file for use in visualSFM in the same folder as the images");
        input128bit_.setDescription("Writes sift file with 128 bit descriptors, for visual SFM compatbility. Only set to false if you know what you are doing.");
    }


    /**
     *
     */
    bool BriskDetectionImpl::execute()
    {
        QString& image        = *dataImage_;
        WSMat& outDescriptors = *dataDescriptor_;
        std::vector<cv::KeyPoint>& keypoints = *dataKeypoint_;
        
        std::vector<cv::KeyPoint> kp;

        const Mat img1 = imread(image.toStdString(), IMREAD_GRAYSCALE);

        //Initialise brisk detector
        Ptr<BRISK> detector = BRISK::create();
        detector->create(*dataThresh_,*dataOctave_);
                
        //Run detector and extract descriptors
        detector->detect(img1, keypoints);
        

        //Extract descriptors
        detector->compute(img1,keypoints,outDescriptors);
                
        if (*dataWriteSIFT_)
        {

            QString siftpre = image.split(".",QString::SkipEmptyParts).at(0);
            
            //Write binary sift file:
            FILE * sift1;
            siftpre.append(".sift");
            sift1 = fopen(siftpre.toLatin1(),"wb");
                    
            int name = ('S' + ('I'<<8) + ('F'<<16) + ('T'<<24));
            int version = ('V' + ('4'<<8) + ('.'<<16)+ ('0'<<24));//Without color
            //int version = ('V' + ('5'<<8) + ('.'<<16)+ ('0'<<24));//With coloer
            int npoint = keypoints.size();
        
            int header[5] = {name, version, npoint, 5, 128};
            fwrite(header, sizeof(int), 5, sift1);
        
            //Location
            for (int i = 0; i < keypoints.size(); ++i)
            {
                float x = keypoints.at(i).pt.x;
                float y = keypoints.at(i).pt.y;
                //Color SOON
                float col = 1;
                float scale = keypoints.at(i).size;
                float orientation = keypoints.at(i).angle;
                float loc[5] = {x,y,col,scale,orientation};
                fwrite(loc, sizeof(float), 5, sift1);
            }
            const int descSize = detector->descriptorSize();

            for (int i = 0; i < outDescriptors.rows; ++i)
            {
                if (*data128bit_)
                {   
                    fwrite(reinterpret_cast<char*>(outDescriptors.row(i).data), sizeof(uchar), 128,sift1);
                    
                }
                else
                {
                    fwrite(reinterpret_cast<char*>(outDescriptors.row(i).data), sizeof(uchar), 64, sift1);//Really bad!!!!Yukyukyuk
                    
                }
            }

            int eof_marker = (0xff+('E'<<8)+('O'<<16)+('F'<<24));
            fwrite(&eof_marker, sizeof(int), 1, sift1);
            fclose(sift1);
        }
        
        return true;
    }


    /**
     *
     */
    BriskDetection::BriskDetection() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< BriskDetection >::getInstance(),
            tr("BRISK Feature detection and extraction"))
    {
        pImpl_ = new BriskDetectionImpl(*this);
    }


    /**
     *
     */
    BriskDetection::~BriskDetection()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  BriskDetection::execute()
    {
        return pImpl_->execute();
    }
}


using namespace RF;
DEFINE_WORKSPACE_OPERATION_FACTORY(BriskDetection, 
                                   RF::StructureFromMotionPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("Structure from motion"))

