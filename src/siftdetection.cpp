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

#include <QImage>
#include <QString>

#include "Workspace/Application/LanguageUtils/streamqstring.h"
#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/InputOutput/inputscalar.h"
#include "Workspace/DataExecution/InputOutput/inputarray.h"
#include "Workspace/DataExecution/InputOutput/output.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"


#include "structurefrommotionplugin.h"
#include "siftdetection.h"
#include "wsmat.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/ocl/ocl.hpp"//ocl
#include "opencv2/nonfree/ocl.hpp"//ocl
#include "opencv2/nonfree/nonfree.hpp"

namespace RF
{
    /**
     * \internal
     */
    using namespace cv;
    class SiftDetectionImpl
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(RF::SiftDetectionImpl)

    public:
        SiftDetection&  op_;

        // Data objects
        CSIRO::DataExecution::TypedObject< QString >  dataImage_;
        //CSIRO::DataExecution::TypedObject< QString >  dataImage2_;
        CSIRO::DataExecution::TypedObject< int >      dataHessianFeats_;
        //CSIRO::DataExecution::TypedObject< double >   dataDistanceRatio_;
        CSIRO::DataExecution::TypedObject< bool >     dataOpenCL_;
        //CSIRO::DataExecution::TypedObject< QImage >  dataOutImage_;
        CSIRO::DataExecution::TypedObject< bool >     dataWriteSIFT_;
        CSIRO::DataExecution::TypedObject<std::vector<cv::KeyPoint> > dataKeypoint_;
        //CSIRO::DataExecution::TypedObject<cv::KeyPoint> dataKeypoint2_;
        CSIRO::DataExecution::TypedObject<WSMat>        dataDescriptor_;
        //CSIRO::DataExecution::TypedObject<WSMat>        dataDescriptor2_;

        // Inputs and outputs
        CSIRO::DataExecution::InputScalar inputImage_;
        //CSIRO::DataExecution::InputScalar inputImage2_;
        CSIRO::DataExecution::InputScalar inputHessianFeats_;
        //CSIRO::DataExecution::InputScalar inputDistanceRatio_;
        CSIRO::DataExecution::InputScalar inputOpenCL_;
        CSIRO::DataExecution::InputScalar inputWriteSIFT_;
        //CSIRO::DataExecution::Output      outputImage_;
        CSIRO::DataExecution::Output       outputKeypoint_;
        //CSIRO::DataExecution::Output       outputKeypoint2_;
        CSIRO::DataExecution::Output       outputDescriptor_;
        //CSIRO::DataExecution::Output       outputDescriptor2_;


        SiftDetectionImpl(SiftDetection& op);

        bool  execute();
        void  logText(const QString& msg)   { op_.logText(msg); }
    };


    /**
     *
     */
    SiftDetectionImpl::SiftDetectionImpl(SiftDetection& op) :
        op_(op),
        dataImage_(),
        //dataImage2_(),
        dataHessianFeats_(400),
        //dataDistanceRatio_(0.8),
        dataOpenCL_(false),
        dataWriteSIFT_(true),
        //dataOutImage_(),
        dataKeypoint_(),
        //dataKeypoint2_(),
        dataDescriptor_(),
        //dataDescriptor2_(),
        inputImage_("Image", dataImage_, op_),
        //inputImage2_("Image2", dataImage2_, op_),
        inputHessianFeats_("Hessian threshold", dataHessianFeats_, op_),
        //inputDistanceRatio_("Lowe distance ratio", dataDistanceRatio_, op_),
        inputOpenCL_("Use openCL feature detection", dataOpenCL_, op_),
        inputWriteSIFT_("Write out SIFT files for VSFM", dataWriteSIFT_, op_),
        //outputImage_("Out Image", dataOutImage_, op_)
        outputKeypoint_("Keypoints", dataKeypoint_, op_),
        outputDescriptor_("Keypoint Descriptors", dataDescriptor_, op_)
    {
        inputWriteSIFT_.setDescription("Writes out a .sift file for use in visualSFM in the same folder as the images");
    }


    /**
     *
     */
    bool SiftDetectionImpl::execute()
    {
        QString& image = *dataImage_;
        //QString& image2 = *dataImage2_;
        //QImage& outimage = *dataOutImage_;
        WSMat& outDescriptors = *dataDescriptor_;
        
        Mat img1 = imread(image.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        //Mat img2 = imread(image2.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        //Mat outimg;
        
        std::vector<KeyPoint>& keypoints = *dataKeypoint_;
        //std::vector<std::vector<DMatch>> matches;
        //std::vector<DMatch> goodmatches;
        
        if (*dataOpenCL_)
        {
            std::cout << QString("WARNING: OpenCL is a bit ropey, tends to crash your video card. Don't blame me if you didn't update your drivers.") + "\n";
            ocl::DeviceType dt = ocl::Context::getContext()->getDeviceInfo().deviceType;
            std::cout << QString("OpenCL device type is %1").arg(dt) + "\n";
            // OCL based SURF detection
            ocl::SURF_OCL detector(*dataHessianFeats_,4,2,true,0.01f,false);
            ocl::oclMat oclimg1;
            //ocl::oclMat oclimg2;
            oclimg1.upload(img1);
            //oclimg2.upload(img2);
        
            ocl::oclMat ocldesc;
            //ocl::oclMat ocldesc2;
            ocl::oclMat mask;
            detector(oclimg1,mask,keypoints,ocldesc);
            ocldesc.download(outDescriptors);
            std::cout << QString("Descriptor byte size is %1").arg(detector.descriptorSize()) + "\n";
            //detector(oclimg2,mask,keypoints2,ocldesc2);
            //Run bruteforce matching
            //ocl::BFMatcher_OCL matcher(NORM_L2);
            //matcher.knnMatch(ocldesc1,ocldesc2,matches,2);
        }
        else
        {
            //Detect features
            SurfFeatureDetector detector(*dataHessianFeats_);
            detector.detect(img1,keypoints);
            //detector.detect(img2,keypoints2);
        
            //Extract descriptions
            SurfDescriptorExtractor extractor(*dataHessianFeats_,4,2,true,0);
            
            extractor.compute(img1, keypoints, outDescriptors);
            std::cout << QString("Descriptor byte size is %1").arg(extractor.descriptorSize()) + "\n";
            //Mat descriptor1, descriptor2;
            //extractor.compute(img1,keypoints1,descriptor1);
            //extractor.compute(img2,keypoints2,descriptor2);
                
            //BFMatcher matcher(NORM_L2);
            //matcher.knnMatch(descriptor1,descriptor2,matches,2);

        }

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
                float loc[5] = {x, y, 1, scale,orientation};
                fwrite(loc, sizeof(float), 5, sift1);
                                
            }
            
            //Descriptor
            std::cout << QString("Number of descriptors is %1, number of keypoints is %2").arg(outDescriptors.rows).arg(keypoints.size()) +"\n";
        
            for (int i = 0; i < outDescriptors.rows; ++i)
            {
                fwrite(&outDescriptors.row(i), sizeof(uchar[128]), 1,sift1);
            }

            int eof_marker = (0xff+('E'<<8)+('O'<<16)+('F'<<24));
            fwrite(&eof_marker, sizeof(int), 1, sift1);
            fclose(sift1);
        }
        
        /*    //Nearest neighbour matching - see Frank Lowe's paper
            for (int i = 0; i < matches.size(); ++i)
            {
                if (matches[i][0].distance < *dataDistanceRatio_ * matches[i][1].distance)
                {
                    goodmatches.push_back(matches[i][0]);
                }
            }
        
        drawMatches(img1, keypoints1, img2, keypoints2,goodmatches,outimg); 
        cvtColor(outimg,outimg,CV_BGR2RGB);
        QImage qtmp((uchar*) outimg.data, outimg.cols, outimg.rows, outimg.step, QImage::Format_RGB888);
        QImage qtmp2(qtmp);
        qtmp2.detach();
        outimage = qtmp2;
        */
       return true;
    }


    /**
     *
     */
    SiftDetection::SiftDetection() :
        CSIRO::DataExecution::Operation(
            CSIRO::DataExecution::OperationFactoryTraits< SiftDetection >::getInstance(),
            tr("SIFT keypoint detection"))
    {
        pImpl_ = new SiftDetectionImpl(*this);
    }


    /**
     *
     */
    SiftDetection::~SiftDetection()
    {
        delete pImpl_;
    }


    /**
     *
     */
    bool  SiftDetection::execute()
    {
        return pImpl_->execute();
    }
}


using namespace RF;
DEFINE_WORKSPACE_OPERATION_FACTORY(SiftDetection, 
                                   RF::StructureFromMotionPlugin::getInstance(),
                                   CSIRO::DataExecution::Operation::tr("Structure from motion"))

