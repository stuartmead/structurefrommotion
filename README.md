Structure-from-Motion plugin for CSIRO Workspace
================================================

This project is a plugin for the [CSIRO Workspace](https://research.csiro.au/workspace/) scientific workflow framework. Its purpose is to utilise open source libraries and methods for photogrammetry and point cloud processing.
This project relies on [OpenCV] (http://opencv.org) and [pointcloudplugin] (https://github.com/csiro-workspace/pointcloudplugin). 

This plugin is still in a development phase, but can be used to perform basic photogrammetry tasks and mesh generation. For more details see the following paper:
Mead, Stuart R., et al. "A Distributed Computing Workflow for Modelling Environmental Flows in Complex Terrain." Environmental Software Systems. Infrastructures, Services and Applications. Springer International Publishing, 2015. 321-332.

Feedback and contributions are more than welcome to help expand its capabilities. 

Contributors
------------
- Stuart Mead -
Risk Frontiers, Dept. of Environmental Sciences, Macquarie University, North Ryde NSW and CSIRO Digital Productivity, Clayton VIC

Compiling and using the plugin
------------------------------
1. Download and install [CSIRO Workspace](https://research.csiro.au/workspace/download/)
3. Checkout the fork of the CSIRO pointcloudplugin repository from [here] (https://github.com/stuartmead/pointcloudplugin) and follow the build instructions.
4. Download and install [OpenCV 2.4] (http://opencv.org) (or build a version yourself), ensure that you add the opencv/bin directory to your PATH variable.
5. Check out this repository
5. Launch CMake from Workspace's Development menu to configure and generate the project. This needs to be done from within Workspace rather than running CMake directly as some key environment variables get setup.
6. Compile the project
7. Add the plugin to Workspace through Settings>Configure application>Plugins
8. Restart Workspace and a Structure from motion group should be visible in the operation catalogue.

Instructions for plugin usage
-----------------------------
Examples and usage instructions are currently being written and will be included in the Wiki.

About CSIRO Workspace
---------------------
Workspace is a powerful scientific application framework and workflow editor which has been in development for over 7 years. Originally designed to construct workflows for scientists in the computational fluid dynamics space, Workspace recognises the importance of interactivity, visualisation, scalability, and ease-of-use in a scientific workflow. It ships with out of the box support for things like mesh processing, interactive 3D visualisation, plotting, networking and database access and is designed to be easily extended through plugins. More than just a workflow editor, Workspace offers a natural path from early code development as part of a research workflow right through to the development of standalone applications for deployment to a collaborator or external client.

Workspace is developed by the Computational Modelling and Simulation Group of Australia’s Commonwealth Scientific and Industrial Research Organisation (CSIRO). Workspace has been developed with support from CSIRO eResearch, Computational and Simulation Sciences and the Digital Productivity Flagship.

For more details or to contact the team see the [Workspace research page](https://research.csiro.au/workspace/).

License
-------
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

Third Party Components
----------------------
The following third party components are distributed with the Software. You agree to comply with the license terms for these components as part of accessing the Software. Other third party software may also be identified in separate files distributed with the Software.

This project is designed to be built against [OpenCV] (http://opencv.org), which is currently licensed under a 3-clause BSD License. Note that this plugin uses SIFT feature detection, which is subject to its own licence for non-commercial use only, see [this page] (http://www.cs.ubc.ca/~lowe/keypoints/) for more details on SIFT.
A more commercial-friendly feature matching algorithim, [BRISK] (http://www.asl.ethz.ch/people/lestefan/personal/BRISK) is included in the project, which is licensed under a 3-clause BSD licence
