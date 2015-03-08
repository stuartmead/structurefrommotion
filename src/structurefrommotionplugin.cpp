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

#include <QString>
#include <QStringList>

#include "Workspace/DataExecution/DataObjects/datafactorytraits.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"

#include "Workspace/DataExecution/DataObjects/typeddatafactory.h"
#include "Workspace/Widgets/enumcomboboxfactory.h"

#include "structurefrommotionplugin.h"
#include "filesindirectory.h"
#include "matchfeatures.h"
#include "wsmat.h"
#include "calibratecamera.h"
#include "briskdetection.h"
#include "siftdetection.h"

#include "opencv2/core/base.hpp"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

namespace RF
{
    /**
     * \internal
     */
    class StructureFromMotionPluginImpl
    {
    public:
        // You can add, remove or modify anything in here without
        // breaking binary compatibility. It starts as empty, but
        // leave it here in case at some time in the future you
        // want to add some data for your plugin without breaking
        // binary compatibility.
    };


    /**
     *
     */
    StructureFromMotionPlugin::StructureFromMotionPlugin() :
            CSIRO::Application::WorkspacePlugin("www.riskfrontiers.com/sfm",
                                                "Structure from Motion",
                                                TOSTRING(STRUCTUREFROMMOTION_PLUGIN_VERSION))
    {
        pImpl_ = new StructureFromMotionPluginImpl;
    }


    /**
     *
     */
    StructureFromMotionPlugin::~StructureFromMotionPlugin()
    {
        delete pImpl_;
    }


    /**
     * \return  The singleton instance of this plugin.
     */
    StructureFromMotionPlugin&  StructureFromMotionPlugin::getInstance()
    {
        // This is a Singleton pattern. There will only ever be one
        // instance of the plugin across the entire application.
        static StructureFromMotionPlugin plugin;
        return plugin;
    }


    /**
     *
     */
    bool  StructureFromMotionPlugin::setup()
    {
        // Add your data factories like this:
        //addFactory( CSIRO::DataExecution::DataFactoryTraits<MyDataType>::getInstance() );
        addFactory(CSIRO::DataExecution::DataFactoryTraits<WSMat>::getInstance());
        addFactory(CSIRO::DataExecution::DataFactoryTraits<cv::KeyPoint>::getInstance());
        addFactory(CSIRO::DataExecution::DataFactoryTraits<std::vector<cv::KeyPoint> >::getInstance());
        addFactory(CSIRO::DataExecution::DataFactoryTraits<std::vector<cv::DMatch> >::getInstance());
        

        // Add your operation factories like this:
        //addFactory( CSIRO::DataExecution::OperationFactoryTraits<MyOperation>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<SiftDetection>::getInstance());
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<BriskDetection>::getInstance());
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<CalibrateCamera>::getInstance());
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<MatchFeatures>::getInstance());
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<FilesInDirectory>::getInstance());

         // Add your widget factories like this:
        //addFactory( MyNamespace::MyWidgetFactory::getInstance() );
        static CSIRO::Widgets::EnumComboBoxFactory<RF::NormTypes> normTypesWidgetFact;
        addFactory(normTypesWidgetFact);


        return true;
    }


    /**
     *
     */
    const CSIRO::DataExecution::OperationFactory*  StructureFromMotionPlugin::getAliasedOperationFactory(const QString& opType) const
    {
        // If you rename an operation, you can provide backwards
        // compatibility using something like this (don't forget to
        // include namespaces in the names if relevant):
        //if (opType == "SomeOperationName")
        //    return &CSIRO::DataExecution::OperationFactoryTraits<NewOperationName>::getInstance();

        // If you make use of opType, you can delete the following Q_UNUSED line
        Q_UNUSED(opType);

        // If we get here, opType is not something we renamed, so return a
        // a null pointer to tell the caller
        return static_cast<const CSIRO::DataExecution::OperationFactory*>(0);
    }


    /**
     *
     */
    const CSIRO::DataExecution::DataFactory*  StructureFromMotionPlugin::getAliasedDataFactory(const QString& dataType) const
    {
        // If you rename a data type, you can provide backwards
        // compatibility using something like this (don't forget to
        // include namespaces in the names if relevant):
        //if (dataType == "SomeDataType")
        //    return &CSIRO::DataExecution::DataFactoryTraits<NewDataType>::getInstance();

        // If you make use of dataType, you can delete the following Q_UNUSED line
        Q_UNUSED(dataType);

        // If we get here, dataType is not something we renamed, so return a
        // a null pointer to tell the caller
        return static_cast<const CSIRO::DataExecution::DataFactory*>(0);
    }
}

#ifndef CSIRO_STATIC_BUILD
extern "C"
{
    CSIRO_EXPORTSPEC CSIRO::Application::WorkspacePlugin* getWorkspacePlugin()
    {
        return &RF::StructureFromMotionPlugin::getInstance();
    }
        /**
     *	\return The version string for the Workspace build we've been built against
     */
    CSIRO_EXPORTSPEC const char* builtAgainstWorkspace()
    {
        #define STRINGIFY(x) #x
        #define TOSTRING(x) STRINGIFY(x)
        return TOSTRING(CSIRO_WORKSPACE_VERSION_CHECK);
    }
}
#endif
