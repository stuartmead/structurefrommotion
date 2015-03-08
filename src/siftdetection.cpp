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

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

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
        CSIRO::DataExecution::TypedObject< int >      dataNfeatures_;
        CSIRO::DataExecution::TypedObject< int >      dataNoctavelayers_;
        CSIRO::DataExecution::TypedObject< double >   dataContrastthreshold_;
        CSIRO::DataExecution::TypedObject< double >   dataEdgethreshold_;
        CSIRO::DataExecution::TypedObject< double >   dataSigma_;
        CSIRO::DataExecution::TypedObject< bool >     dataWriteSIFT_;
        CSIRO::DataExecution::TypedObject<std::vector<cv::KeyPoint> > dataKeypoint_;
        CSIRO::DataExecution::TypedObject<WSMat>        dataDescriptor_;

        // Inputs and outputs
        CSIRO::DataExecution::InputScalar inputImage_;
        CSIRO::DataExecution::InputScalar inputNfeatures_;
        CSIRO::DataExecution::InputScalar inputNoctaveLayers_;
        CSIRO::DataExecution::InputScalar inputContrastthreshold_;
        CSIRO::DataExecution::InputScalar inputEdgethreshold_;
        CSIRO::DataExecution::InputScalar inputSigma_;
        CSIRO::DataExecution::InputScalar inputWriteSIFT_;
        CSIRO::DataExecution::Output       outputKeypoint_;
        CSIRO::DataExecution::Output       outputDescriptor_;


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
        dataNfeatures_(0),
        dataNoctavelayers_(3),
        dataContrastthreshold_(0.04),
        dataEdgethreshold_(10),
        dataSigma_(1.6),
        dataWriteSIFT_(false),
        dataKeypoint_(),
        dataDescriptor_(),
        inputImage_("Image", dataImage_, op_),
        inputNfeatures_("Number of features", dataNfeatures_, op_),
        inputNoctaveLayers_("Number of octave layers", dataNoctavelayers_, op_),
        inputContrastthreshold_("Contrast Threshold", dataContrastthreshold_, op_),
        inputEdgethreshold_("Edge Threshold", dataEdgethreshold_, op_),
        inputSigma_("Gaussian sigma", dataSigma_, op_),
        inputWriteSIFT_("Write out SIFT files for VSFM", dataWriteSIFT_, op_),
        outputKeypoint_("Keypoints", dataKeypoint_, op_),
        outputDescriptor_("Keypoint Descriptors", dataDescriptor_, op_)
    {
        inputNfeatures_.setDescription("The number of best features to retain. The features are ranked by their scores \
                                       (measured in SIFT algorithm as the local contrast)");
        inputNoctaveLayers_.setDescription("The number of layers in each octave. 3 is the value used in D. Lowe paper. \
                                           The number of octaves is computed automatically from the image resolution.");
        inputContrastthreshold_.setDescription("The contrast threshold used to filter out weak features in semi-uniform \
                                               (low-contrast) regions. The larger the threshold, the less features are produced by the detector.");
        inputEdgethreshold_.setDescription("The threshold used to filter out edge-like features. Note that the its meaning is different from \
                                          the contrastThreshold, i.e. the larger the edgeThreshold, the less features are filtered out (more features are retained).");
        inputSigma_.setDescription("The sigma of the Gaussian applied to the input image at the octave \#0. If your image \
                                   is captured with a weak camera with soft lenses, you might want to reduce the number.");
        inputWriteSIFT_.setDescription("Writes out a .sift file for use in visualSFM in the same folder as the images");
    }


    /**
     *
     */
    bool SiftDetectionImpl::execute()
    {
        QString& image = *dataImage_;
        WSMat& outDescriptors = *dataDescriptor_;
        
        Mat img1 = imread(image.toStdString(), IMREAD_GRAYSCALE);
                
        std::vector<KeyPoint>& keypoints = *dataKeypoint_;
        
        
        //Detect features
        cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create(*dataNfeatures_,*dataNoctavelayers_,*dataContrastthreshold_,*dataEdgethreshold_,*dataSigma_);

        detector->detect(img1,keypoints);
            
        //Extract descriptions
        detector->compute(img1, keypoints, outDescriptors);

        std::cout << QString("Descriptor byte size is %1").arg(detector->descriptorSize()) + "\n";
        std::cout << QString("Number of descriptors is %1, number of keypoints is %2").arg(outDescriptors.rows).arg(keypoints.size()) +"\n";
        

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

