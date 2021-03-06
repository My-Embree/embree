// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#ifdef _WIN32
#  define RTC_API extern "C" __declspec(dllexport)
#else
#  define RTC_API extern "C" __attribute__ ((visibility ("default")))
#endif

#include "default.h"
#include "device.h"
#include "scene.h"
#include "context.h"
#include "../../include/embree3/rtcore_ray.h"
//new
#include "../../kernels/bvh/bvh.h"
#include "../../kernels/geometry/trianglev.h"
#include "c:\Users\evanwaxman\Documents\My-Embree\embree\kernels\common\accel.h"
#include <queue>
#include <iostream>
#include <fstream>
#include "c:\Users\evanwaxman\Documents\My-Embree\embree\kernels\common\scene_instance.h"
#include "c:\Users\evanwaxman\Documents\My-Embree\embree\tutorials\common\tutorial\tutorial_device.h"
#include "c:\Users\evanwaxman\Documents\My-Embree\embree\kernels\geometry\triangle.h"
#include "c:\Users\evanwaxman\Documents\My-Embree\embree\kernels\geometry\curveNv.h"
#include "c:\Users\evanwaxman\Documents\My-Embree\embree\kernels\subdiv\bezier_curve.h"


//#define PRINT_CONSOLE
#define SKIP

namespace embree
{  
  /* mutex to make API thread safe */
  static MutexSys g_mutex;

  RTC_API RTCDevice rtcNewDevice(const char* config)
  {
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcNewDevice);
    Lock<MutexSys> lock(g_mutex);
    Device* device = new Device(config);
    return (RTCDevice) device->refInc();
    RTC_CATCH_END(nullptr);
    return (RTCDevice) nullptr;
  }

  RTC_API void rtcRetainDevice(RTCDevice hdevice) 
  {
    Device* device = (Device*) hdevice;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcRetainDevice);
    RTC_VERIFY_HANDLE(hdevice);
    Lock<MutexSys> lock(g_mutex);
    device->refInc();
    RTC_CATCH_END(nullptr);
  }
  
  RTC_API void rtcReleaseDevice(RTCDevice hdevice) 
  {
    Device* device = (Device*) hdevice;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcReleaseDevice);
    RTC_VERIFY_HANDLE(hdevice);
    Lock<MutexSys> lock(g_mutex);
    device->refDec();
    RTC_CATCH_END(nullptr);
  }
  
  RTC_API ssize_t rtcGetDeviceProperty(RTCDevice hdevice, RTCDeviceProperty prop)
  {
    Device* device = (Device*) hdevice;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetDeviceProperty);
    RTC_VERIFY_HANDLE(hdevice);
    Lock<MutexSys> lock(g_mutex);
    return device->getProperty(prop);
    RTC_CATCH_END(device);
    return 0;
  }

  RTC_API void rtcSetDeviceProperty(RTCDevice hdevice, const RTCDeviceProperty prop, ssize_t val)
  {
    Device* device = (Device*) hdevice;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetDeviceProperty);
    const bool internal_prop = (size_t)prop >= 1000000 && (size_t)prop < 1000004;
    if (!internal_prop) RTC_VERIFY_HANDLE(hdevice); // allow NULL device for special internal settings
    Lock<MutexSys> lock(g_mutex);
    device->setProperty(prop,val);
    RTC_CATCH_END(device);
  }

  RTC_API RTCError rtcGetDeviceError(RTCDevice hdevice)
  {
    Device* device = (Device*) hdevice;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetDeviceError);
    if (device == nullptr) return Device::getThreadErrorCode();
    else                   return device->getDeviceErrorCode();
    RTC_CATCH_END(device);
    return RTC_ERROR_UNKNOWN;
  }

  RTC_API void rtcSetDeviceErrorFunction(RTCDevice hdevice, RTCErrorFunction error, void* userPtr)
  {
    Device* device = (Device*) hdevice;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetDeviceErrorFunction);
    RTC_VERIFY_HANDLE(hdevice);
    device->setErrorFunction(error, userPtr);
    RTC_CATCH_END(device);
  }

  RTC_API void rtcSetDeviceMemoryMonitorFunction(RTCDevice hdevice, RTCMemoryMonitorFunction memoryMonitor, void* userPtr)
  {
    Device* device = (Device*) hdevice;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetDeviceMemoryMonitorFunction);
    device->setMemoryMonitorFunction(memoryMonitor, userPtr);
    RTC_CATCH_END(device);
  }

  RTC_API RTCBuffer rtcNewBuffer(RTCDevice hdevice, size_t byteSize)
  {
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcNewBuffer);
    RTC_VERIFY_HANDLE(hdevice);
    Buffer* buffer = new Buffer((Device*)hdevice, byteSize);
    return (RTCBuffer)buffer->refInc();
    RTC_CATCH_END((Device*)hdevice);
    return nullptr;
  }

  RTC_API RTCBuffer rtcNewSharedBuffer(RTCDevice hdevice, void* ptr, size_t byteSize)
  {
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcNewSharedBuffer);
    RTC_VERIFY_HANDLE(hdevice);
    Buffer* buffer = new Buffer((Device*)hdevice, byteSize, ptr);
    return (RTCBuffer)buffer->refInc();
    RTC_CATCH_END((Device*)hdevice);
    return nullptr;
  }

  RTC_API void* rtcGetBufferData(RTCBuffer hbuffer)
  {
    Buffer* buffer = (Buffer*)hbuffer;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetBufferData);
    RTC_VERIFY_HANDLE(hbuffer);
    return buffer->data();
    RTC_CATCH_END2(buffer);
    return nullptr;
  }

  RTC_API void rtcRetainBuffer(RTCBuffer hbuffer)
  {
    Buffer* buffer = (Buffer*)hbuffer;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcRetainBuffer);
    RTC_VERIFY_HANDLE(hbuffer);
    buffer->refInc();
    RTC_CATCH_END2(buffer);
  }
  
  RTC_API void rtcReleaseBuffer(RTCBuffer hbuffer)
  {
    Buffer* buffer = (Buffer*)hbuffer;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcReleaseBuffer);
    RTC_VERIFY_HANDLE(hbuffer);
    buffer->refDec();
    RTC_CATCH_END2(buffer);
  }

  RTC_API RTCScene rtcNewScene (RTCDevice hdevice) 
  {
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcNewScene);
    RTC_VERIFY_HANDLE(hdevice);
    Scene* scene = new Scene((Device*)hdevice);
    return (RTCScene) scene->refInc();
    RTC_CATCH_END((Device*)hdevice);
    return nullptr;
  }

  RTC_API void rtcSetSceneProgressMonitorFunction(RTCScene hscene, RTCProgressMonitorFunction progress, void* ptr) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetSceneProgressMonitorFunction);
    RTC_VERIFY_HANDLE(hscene);
    Lock<MutexSys> lock(g_mutex);
    scene->setProgressMonitorFunction(progress,ptr);
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcSetSceneBuildQuality (RTCScene hscene, RTCBuildQuality quality) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetSceneBuildQuality);
    RTC_VERIFY_HANDLE(hscene);
    if (quality != RTC_BUILD_QUALITY_LOW &&
        quality != RTC_BUILD_QUALITY_MEDIUM &&
        quality != RTC_BUILD_QUALITY_HIGH)
      throw std::runtime_error("invalid build quality");
    scene->setBuildQuality(quality);
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcSetSceneFlags (RTCScene hscene, RTCSceneFlags flags) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetSceneFlags);
    RTC_VERIFY_HANDLE(hscene);
    scene->setSceneFlags(flags);
    RTC_CATCH_END2(scene);
  }

  RTC_API RTCSceneFlags rtcGetSceneFlags(RTCScene hscene)
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetSceneFlags);
    RTC_VERIFY_HANDLE(hscene);
    return scene->getSceneFlags();
    RTC_CATCH_END2(scene);
    return RTC_SCENE_FLAG_NONE;
  }

  RTC_API void rtcCommitScene (RTCScene hscene) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcCommitScene);
    RTC_VERIFY_HANDLE(hscene);
	scene->commit(false);


#ifndef SKIP

	
#ifdef PRINT_CONSOLE
	//=================================================================================================================
	BVH4* bvh4 = nullptr;
	BVH8* bvh8 = nullptr;
	BVH4::UnalignedNode* un4 = nullptr;
	BVH8::UnalignedNode* un8 = nullptr;
	BVH4::AlignedNode* n4 = nullptr;
	BVH8::AlignedNode* n8 = nullptr;
	std::queue<BVH4::NodeRef> nodeQueue4;
	std::queue<BVH8::NodeRef> nodeQueue8;
	std::queue<unsigned long long> idQueue;
	std::queue<BBox3fa> boundsQueue;		// for leaf bounds

	/* if the scene contains only triangles, the BVH4 acceleration structure can be obtained this way */
		AccelData* accel = ((Accel*)scene)->intersectors.ptr;
		//=================================================BVH4 ONLY==========================================
		if (accel->type == AccelData::TY_BVH4) {
			bvh4 = (BVH4*)accel;
			BVH4::NodeRef node = bvh4->root;
			std::cout << "TY_BVH4:\n";

			// initialize queue with root node and root id
			nodeQueue4.push(node);
			idQueue.push(0);
			while (!(nodeQueue4.empty())) {
				BVH4::NodeRef tempNode = nodeQueue4.front();
				nodeQueue4.pop();
				unsigned long long tempID = idQueue.front();
				idQueue.pop();
				//std::cout << "\tNodeID: ";

				//leaf node
				if (tempNode.type() == 9 || tempNode.type() == 10) {
					std::cout << "\tNodeID: " << tempID << ", Node Type: Leaf Node (" << tempNode.type() << ")\n";

					size_t num;
					BBox3fa b;

					b = boundsQueue.front();
					boundsQueue.pop();

					// change primID to equal leaf node id
					if (scene->geometries[0]->gtype == 16) {		// GTY_TRIANGLE_MESH
						Triangle4v* tri = (Triangle4v*)tempNode.leaf(num);
						Triangle4* tri2 = (Triangle4*)tempNode.leaf(num);

						for (size_t i = 0; i < num; i++) {
							for (size_t j = 0; j < tri2[i].size(); j++) {
								std::cout << "\t\t\tGeomID: " << tri2[i].geomID(j) << ", PrimID: " << tri2[i].primID(j) <<
									", Bounds: [(" << tri2[i].v0.x[j] << ", " << tri2[i].v0.y[j] << ", " << tri2[i].v0.z[j] <<
									"); (" << tri2[i].v0.x[j] - tri2[i].e1.x[j] << ", " << tri2[i].v0.y[j] - tri2[i].e1.y[j] << ", " << tri2[i].v0.z[j] - tri2[i].e1.z[j] <<
									"); (" << tri2[i].v0.x[j] + tri2[i].e2.x[j] << ", " << tri2[i].v0.y[j] + tri2[i].e2.y[j] << ", " << tri2[i].v0.z[j] + tri2[i].e2.z[j] << ")]\n";
							}
						}
					}
					else if (scene->geometries[0]->gtype == 4) {	// GTY_FLAT_BEZIER_CURVE
						//const PrimRef& prim = (PrimRef)tempNode.leaf(num);
						Curve4v* curve = (Curve4v*)tempNode.leaf(num);
						BezierCurve3fa* bezCurve = (BezierCurve3fa*)tempNode.leaf(num);
						QuadraticBezierCurve3fa* quadBezCurve = (QuadraticBezierCurve3fa*)tempNode.leaf(num);

						//std::cout << "\t\t\tGeomID: " << curve->geomID(0) << ", PrimID: " << curve->primID << ", Bounds: ";
							//prim->bounds() << "\n";
					}
					else {
						std::cout << "UNKNOWN GEOM TYPE\n";
					}
				}
				//aligned node (inner node)
				else if (tempNode.type() == 0) {
					std::cout << "\tNodeID: " << tempID << ", Node Type: Aligned Node (" << tempNode.type() << ")\n";
					n4 = tempNode.alignedNode();
					vector <float> lowerX;
					vector <float> lowerY;
					vector <float> lowerZ;
					vector <float> upperX;
					vector <float> upperY;
					vector <float> upperZ;
					unsigned int numChildren = 0;
					for (int i = 0; i < 4; i++) {
						if (n4->bounds(i).lower.x == (float)pos_inf || n4->bounds(i).lower.x == (float)neg_inf) break;
						lowerX.push_back(n4->bounds(i).lower.x);
						lowerY.push_back(n4->bounds(i).lower.y);
						lowerZ.push_back(n4->bounds(i).lower.z);
						upperX.push_back(n4->bounds(i).upper.x);
						upperY.push_back(n4->bounds(i).upper.y);
						upperZ.push_back(n4->bounds(i).upper.z);
						numChildren++;

						std::cout << "\t\tChild " << i << " bounds: " << n4->bounds(i) << ", NodeID: " << tempID * 4 + (i + 1) << "\n";
					}

					// push children to queue
					for (int i = 0; i < 4; i++) {
						nodeQueue4.push(n4->child(i));
						idQueue.push(tempID * 4 + (i + 1));

						// push leaf node bounds onto queue to access when current node is leaf node
						if (n4->child(i).type() == 9 || n4->child(i).type() == 10) {	// valid leaf node
							boundsQueue.push(n4->bounds(i));
						}
					}
				}
				else if (tempNode.type() == 8) {
					std::cout << "\tEMPTY NODE" << "\n";
				}
				else {
					std::cout << "ERROR: UNKNOWN NODE TYPE " << tempNode.type() << "\n";
				}
			}
		}
		//===================================BVH8 ONLY=============================
		else if (accel->type == AccelData::TY_BVH8) {
			bvh8 = (BVH8*)accel;
			BVH8::NodeRef node = bvh8->root;
			std::cout << "TY_BVH8:\n";

			// initialize queue with root node and root id
			nodeQueue8.push(node);
			idQueue.push(0);
			while (!(nodeQueue8.empty())) {
				BVH8::NodeRef tempNode = nodeQueue8.front();
				nodeQueue4.pop();
				unsigned long long tempID = idQueue.front();
				idQueue.pop();
				//std::cout << "\tNodeID: ";

				//leaf node
				if (tempNode.type() == 9 || tempNode.type() == 10) {
					std::cout << "\tNodeID: " << tempID << ", Node Type: Leaf Node (" << tempNode.type() << ")\n";

					size_t num;
					BBox3fa b;

					b = boundsQueue.front();
					boundsQueue.pop();

					// change primID to equal leaf node id
					if (scene->geometries[0]->gtype == 16) {		// GTY_TRIANGLE_MESH
						Triangle4v* tri = (Triangle4v*)tempNode.leaf(num);
						Triangle4* tri2 = (Triangle4*)tempNode.leaf(num);

						for (size_t i = 0; i < num; i++) {
							for (size_t j = 0; j < tri2[i].size(); j++) {
								std::cout << "\t\t\tGeomID: " << tri2[i].geomID(j) << ", PrimID: " << tri2[i].primID(j) <<
									", Bounds: [(" << tri2[i].v0.x[j] << ", " << tri2[i].v0.y[j] << ", " << tri2[i].v0.z[j] <<
									"); (" << tri2[i].v0.x[j] - tri2[i].e1.x[j] << ", " << tri2[i].v0.y[j] - tri2[i].e1.y[j] << ", " << tri2[i].v0.z[j] - tri2[i].e1.z[j] <<
									"); (" << tri2[i].v0.x[j] + tri2[i].e2.x[j] << ", " << tri2[i].v0.y[j] + tri2[i].e2.y[j] << ", " << tri2[i].v0.z[j] + tri2[i].e2.z[j] << ")]\n";
							}
						}
					}
					else if (scene->geometries[0]->gtype == 4) {	// GTY_FLAT_BEZIER_CURVE
						PrimRef* prim = (PrimRef*)tempNode.leaf(num);
						Curve4v* curve = (Curve4v*)tempNode.leaf(num);
						BezierCurve3fa* bezCurve = (BezierCurve3fa*)tempNode.leaf(num);
						QuadraticBezierCurve3fa* quadBezCurve = (QuadraticBezierCurve3fa*)tempNode.leaf(num);

						//std::cout << "\t\t\tGeomID: " << curve->geomID << ", PrimID: " << curve->primID << ", Bounds: " <<
						//	prim->bounds() << "\n";
					}
					else {
						std::cout << "UNKNOWN GEOM TYPE\n";
					}

				}
				//aligned node (inner node)
				else if (tempNode.type() == 0) {
					std::cout << "\tNodeID: " << tempID << ", Node Type: Aligned Node (" << tempNode.type() << ")\n";
					n8 = tempNode.alignedNode();
					vector <float> lowerX;
					vector <float> lowerY;
					vector <float> lowerZ;
					vector <float> upperX;
					vector <float> upperY;
					vector <float> upperZ;
					unsigned int numChildren = 0;
					for (int i = 0; i < 8; i++) {
						if (n8->bounds(i).lower.x == (float)pos_inf || n8->bounds(i).lower.x == (float)neg_inf) break;
						lowerX.push_back(n8->bounds(i).lower.x);
						lowerY.push_back(n8->bounds(i).lower.y);
						lowerZ.push_back(n8->bounds(i).lower.z);
						upperX.push_back(n8->bounds(i).upper.x);
						upperY.push_back(n8->bounds(i).upper.y);
						upperZ.push_back(n8->bounds(i).upper.z);
						numChildren++;

						std::cout << "\t\tChild " << i << " bounds: " << n8->bounds(i) << ", NodeID: " << tempID * 8 + (i + 1) << "\n";
					}

					// push children to queue
					for (int i = 0; i < 8; i++) {
						nodeQueue8.push(n8->child(i));
						idQueue.push(tempID * 8 + (i + 1));

						// push leaf node bounds onto queue to access when current node is leaf node
						if (n8->child(i).type() == 9 || n8->child(i).type() == 10) {	// valid leaf node
							boundsQueue.push(n8->bounds(i));
						}
					}
				}
				else if (tempNode.type() == 8) {
					std::cout << "\tEMPTY NODE" << "\n";
				}
				else {
					std::cout << "ERROR: UNKNOWN NODE TYPE " << tempNode.type() << "\n";
				}
			}
		}
		/* if there are also other geometry types, one has to iterate over the toplevel AccelN structure */
		//============================================= BOTH ==============================================================
		else if (accel->type == AccelData::TY_ACCELN)
		{
			/*
			std::cout << scene->geometries[0]->quality << "\n";
			std::cout << scene->geometries[1]->quality << "\n";
			std::cout << scene->geometries[2]->quality << "\n";
			std::cout << scene->geometries[0]->geomID << "\n";
			std::cout << scene->geometries[1]->geomID << "\n";
			std::cout << scene->geometries[2]->geomID << "\n";

			AccelN* test = ((AccelN*)scene->geometries[1]->scene->intersectors.ptr);
			*/
			AccelN* accelN = (AccelN*)(accel);
			std::cout << "Top level BVH consists of:\n";
			for (size_t i = 0; i < accelN->validAccels.size(); i++) {
				if (accelN->validAccels[i]->intersectors.ptr->type == AccelData::TY_BVH4) {
					bvh4 = (BVH4*)accelN->validAccels[i]->intersectors.ptr;
					BVH4::NodeRef node = bvh4->root;
					std::cout << "BVH type: TY_BVH4, Geometry type: " << scene->geometries[i]->gtype << "\n";


					// access instance associated with current bvh
					Instance* inst = ((Instance*)bvh4->scene->geometries[i].ptr);

					//std::cout << "\tLocal to World Transformation Matrix: \n";
					//std::cout << "\t\tvx = " << inst->local2world->l.vx << "\n";
					//std::cout << "\t\tvy = " << inst->local2world->l.vy << "\n";
					//std::cout << "\t\tvz = " << inst->local2world->l.vz << "\n";


					// initialize queue with root node and root id
					nodeQueue4.push(node);
					idQueue.push(0);
					while (!(nodeQueue4.empty())) {
						while (!(nodeQueue4.empty())) {
							BVH4::NodeRef tempNode = nodeQueue4.front();
							nodeQueue4.pop();
							unsigned long long tempID = idQueue.front();
							idQueue.pop();
							//std::cout << "\tNodeID: ";

							//leaf node
							if (tempNode.type() == 9 || tempNode.type() == 10) {
								std::cout << "\tNodeID: " << tempID << ", Node Type: Leaf Node (" << tempNode.type() << ")\n";

								size_t num;
								BBox3fa b;

								b = boundsQueue.front();
								boundsQueue.pop();
								
								// change primID to equal leaf node id
								if (inst->gtype == 16) {		// GTY_TRIANGLE_MESH
									Triangle4v* tri = (Triangle4v*)tempNode.leaf(num);
									Triangle4* tri2 = (Triangle4*)tempNode.leaf(num);

									for (size_t i = 0; i < num; i++) {
										for (size_t j = 0; j < tri2[i].size(); j++) {
											std::cout << "\t\t\tGeomID: " << tri2[i].geomID(j) << ", PrimID: " << tri2[i].primID(j) <<
												", Bounds: [(" << tri2[i].v0.x[j] << ", " << tri2[i].v0.y[j] << ", " << tri2[i].v0.z[j] <<
												"); (" << tri2[i].v0.x[j] - tri2[i].e1.x[j] << ", " << tri2[i].v0.y[j] - tri2[i].e1.y[j] << ", " << tri2[i].v0.z[j] - tri2[i].e1.z[j] <<
												"); (" << tri2[i].v0.x[j] + tri2[i].e2.x[j] << ", " << tri2[i].v0.y[j] + tri2[i].e2.y[j] << ", " << tri2[i].v0.z[j] + tri2[i].e2.z[j] << ")]\n";
										}
									}
								}
								else if (inst->gtype == 4) {	// GTY_FLAT_BEZIER_CURVE
									PrimRef* prim = (PrimRef*)tempNode.leaf(num);
									Curve4v* curve = (Curve4v*)tempNode.leaf(num);
									BezierCurve3fa* bezCurve = (BezierCurve3fa*)tempNode.leaf(num);
									QuadraticBezierCurve3fa* quadBezCurve = (QuadraticBezierCurve3fa*)tempNode.leaf(num);
									
									//std::cout << "\t\t\tGeomID: " << curve->geomID << ", PrimID: " << curve->primID << ", Bounds: " <<
									//	prim->bounds() << "\n";
								}
							}
							//aligned node (inner node)
							else if (tempNode.type() == 0) {
								std::cout << "\tNodeID: " << tempID << ", Node Type: Aligned Node (" << tempNode.type() << ")\n";
								n4 = tempNode.alignedNode();
								vector <float> lowerX;
								vector <float> lowerY;
								vector <float> lowerZ;
								vector <float> upperX;
								vector <float> upperY;
								vector <float> upperZ;
								unsigned int numChildren = 0;
								for (int i = 0; i < 4; i++) {
									if (n4->bounds(i).lower.x == (float)pos_inf || n4->bounds(i).lower.x == (float)neg_inf) break;
									lowerX.push_back(n4->bounds(i).lower.x);
									lowerY.push_back(n4->bounds(i).lower.y);
									lowerZ.push_back(n4->bounds(i).lower.z);
									upperX.push_back(n4->bounds(i).upper.x);
									upperY.push_back(n4->bounds(i).upper.y);
									upperZ.push_back(n4->bounds(i).upper.z);
									numChildren++;

									std::cout << "\t\tChild " << i << " bounds: " << n4->bounds(i) << ", NodeID: " << tempID * 4 + (i + 1) << "\n";
								}

								// push children to queue
								for (int i = 0; i < 4; i++) {
									nodeQueue4.push(n4->child(i));
									idQueue.push(tempID * 4 + (i + 1));

									// push leaf node bounds onto queue to access when current node is leaf node
									if (n4->child(i).type() == 9 || n4->child(i).type() == 10) {	// valid leaf node
										boundsQueue.push(n4->bounds(i));
									}
								}
							}
							// unaligned node
							else if (tempNode.isUnalignedNode()) {
								std::cout << "\tNodeID: " << tempID << ", Node Type: Unaligned Node (" << tempNode.type() << ")\n";
								un4 = tempNode.unalignedNode();
								vector <float> lowerX;
								vector <float> lowerY;
								vector <float> lowerZ;
								vector <float> upperX;
								vector <float> upperY;
								vector <float> upperZ;
								/*
								int numChildren = 0;
								for (int i = 0; i < 8; i++) {

									if (un4->bounds(i).lower.x == (float)pos_inf || un4->bounds(i).lower.x == (float)neg_inf) break;
									lowerX.push_back(n4->bounds(i).lower.x);
									lowerY.push_back(n4->bounds(i).lower.y);
									lowerZ.push_back(n4->bounds(i).lower.z);
									upperX.push_back(n4->bounds(i).upper.x);
									upperY.push_back(n4->bounds(i).upper.y);
									upperZ.push_back(n4->bounds(i).upper.z);
									numChildren++;

									std::cout << "\t\tChild " << i << " bounds: " << n4->bounds(i) << ", NodeID: " << tempID * 4 + (i + 1) << "\n";
								}
								*/

								//std::cout << "Bounding Box: " << un8->naabb << "\n";

								// push children to queue
								for (int i = 0; i < 4; i++) {
									nodeQueue4.push(un4->child(i));
									idQueue.push(tempID * 4 + (i + 1));

									// push leaf node bounds onto queue to access when current node is leaf node
									if (un4->child(i).type() == 9 || un4->child(i).type() == 10) {	// valid leaf node
										//boundsQueue.push(un4->bounds(i));
									}
								}

							}
							else if (tempNode.type() == 8) {
								std::cout << "\tEMPTY NODE" << "\n";
							}
							else {
								std::cout << "ERROR: UNKNOWN NODE TYPE " << tempNode.type() << "\n";
							}
						}
					}
				}
				else if (accelN->validAccels[i]->intersectors.ptr->type == AccelData::TY_BVH8) {
					bvh8 = (BVH8*)accelN->validAccels[i]->intersectors.ptr;
					BVH8::NodeRef node = bvh8->root;

					std::cout << "BVH type: TY_BVH8, Geometry type: " << scene->geometries[i]->gtype << "\n";

					// access instance associated with current bvh
					Instance* inst = ((Instance*)bvh8->scene->geometries[0].ptr);

					//std::cout << "\tLocal to World Transformation Matrix: \n";
					//std::cout << "\t\tvx = " << inst->local2world->l.vx << "\n";
					//std::cout << "\t\tvy = " << inst->local2world->l.vy << "\n";
					//std::cout << "\t\tvz = " << inst->local2world->l.vz << "\n";

					// initialize queue with root node and root id
					nodeQueue8.push(node);
					idQueue.push(0);
					while (!(nodeQueue8.empty())) {
						BVH8::NodeRef tempNode = nodeQueue8.front();
						nodeQueue8.pop();
						unsigned long long tempID = idQueue.front();
						idQueue.pop();
						//std::cout << "\tNodeID: ";

						//leaf node
						if (tempNode.type() == 9 || tempNode.type() == 10) {
							std::cout << "\tNodeID: " << tempID << ", Node Type: Leaf Node (" << tempNode.type() << ")\n";

							size_t num;
							BBox3fa b;

							b = boundsQueue.front();
							boundsQueue.pop();

							// change primID to equal leaf node id
							if (scene->geometries[i]->gtype == 16) {		// GTY_TRIANGLE_MESH
								Triangle4v* tri = (Triangle4v*)tempNode.leaf(num);
								Triangle4* tri2 = (Triangle4*)tempNode.leaf(num);

								for (size_t i = 0; i < num; i++) {
									for (size_t j = 0; j < tri2[i].size(); j++) {
										std::cout << "\t\t\tGeomID: " << tri2[i].geomID(j) << ", PrimID: " << tri2[i].primID(j) <<
											", Bounds: [(" << tri2[i].v0.x[j] << ", " << tri2[i].v0.y[j] << ", " << tri2[i].v0.z[j] <<
											"); (" << tri2[i].v0.x[j] - tri2[i].e1.x[j] << ", " << tri2[i].v0.y[j] - tri2[i].e1.y[j] << ", " << tri2[i].v0.z[j] - tri2[i].e1.z[j] <<
											"); (" << tri2[i].v0.x[j] + tri2[i].e2.x[j] << ", " << tri2[i].v0.y[j] + tri2[i].e2.y[j] << ", " << tri2[i].v0.z[j] + tri2[i].e2.z[j] << ")]\n";
									}
								}
							}
							else if (scene->geometries[i]->gtype == 4) {	// GTY_FLAT_BEZIER_CURVE
								PrimRef* prim = (PrimRef*)tempNode.leaf(num);
								Curve4v* curve = (Curve4v*)tempNode.leaf(num);
								BezierCurve3fa* bezCurve = (BezierCurve3fa*)tempNode.leaf(num);
								QuadraticBezierCurve3fa* quadBezCurve = (QuadraticBezierCurve3fa*)tempNode.leaf(num);

								//std::cout << "\t\t\tGeomID: " << curve->geomID << ", PrimID: " << curve->primID << ", Bounds: " <<
								//	prim->bounds() << "\n";
							}
						}
						//aligned node (inner node)
						else if (tempNode.type() == 0) {
							std::cout << "\tNodeID: " << tempID << ", Node Type: Aligned Node (" << tempNode.type() << ")\n";
							n8 = tempNode.alignedNode();
							vector <float> lowerX;
							vector <float> lowerY;
							vector <float> lowerZ;
							vector <float> upperX;
							vector <float> upperY;
							vector <float> upperZ;
							unsigned int numChildren = 0;
							for (int i = 0; i < 8; i++) {
								if (n8->bounds(i).lower.x == (float)pos_inf || n8->bounds(i).lower.x == (float)neg_inf) break;
								lowerX.push_back(n8->bounds(i).lower.x);
								lowerY.push_back(n8->bounds(i).lower.y);
								lowerZ.push_back(n8->bounds(i).lower.z);
								upperX.push_back(n8->bounds(i).upper.x);
								upperY.push_back(n8->bounds(i).upper.y);
								upperZ.push_back(n8->bounds(i).upper.z);
								numChildren++;

								std::cout << "\t\tChild " << i << " bounds: " << n8->bounds(i) << ", NodeID: " << tempID * 8 + (i + 1) << "\n";
							}

							// push children to queue
							for (int i = 0; i < 8; i++) {
								nodeQueue8.push(n8->child(i));
								idQueue.push(tempID * 8 + (i + 1));

								// push leaf node bounds onto queue to access when current node is leaf node
								if (n8->child(i).type() == 9 || n8->child(i).type() == 10) {	// valid leaf node
									boundsQueue.push(n8->bounds(i));
								}
							}
						}
						// unaligned node
						else if (tempNode.isUnalignedNode()) {
							std::cout << "\tNodeID: " << tempID << ", Node Type: Unaligned Node (" << tempNode.type() << ")\n";
							un8 = tempNode.unalignedNode();
							vector <float> lowerX;
							vector <float> lowerY;
							vector <float> lowerZ;
							vector <float> upperX;
							vector <float> upperY;
							vector <float> upperZ;

							/*
							int numChildren = 0;
							for (int i = 0; i < 8; i++) {
								
								if (un4->bounds(i).lower.x == (float)pos_inf || un4->bounds(i).lower.x == (float)neg_inf) break;
								lowerX.push_back(n4->bounds(i).lower.x);
								lowerY.push_back(n4->bounds(i).lower.y);
								lowerZ.push_back(n4->bounds(i).lower.z);
								upperX.push_back(n4->bounds(i).upper.x);
								upperY.push_back(n4->bounds(i).upper.y);
								upperZ.push_back(n4->bounds(i).upper.z);
								numChildren++;

								std::cout << "\t\tChild " << i << " bounds: " << n4->bounds(i) << ", NodeID: " << tempID * 4 + (i + 1) << "\n";
							}
							*/

							//std::cout << "Bounding Box: " << un8->naabb << "\n";

							// push children to queue
							for (int i = 0; i < 4; i++) {
								nodeQueue4.push(un4->child(i));
								idQueue.push(tempID * 4 + (i + 1));

								// push leaf node bounds onto queue to access when current node is leaf node
								if (un4->child(i).type() == 9 || un4->child(i).type() == 10) {	// valid leaf node
																								//boundsQueue.push(un4->bounds(i));
								}
							}
						}
						else if (tempNode.type() == 8) {
							std::cout << "\tEMPTY NODE" << "\n";
						}
						else {
							std::cout << "ERROR: UNKNOWN NODE TYPE " << tempNode.type() << "\n";
						}
					}
				}
			}
		}
#else
//=================================================================================================================
	BVH4* bvh4 = nullptr;
	BVH8* bvh8 = nullptr;
	BVH4::AlignedNode* n4 = nullptr;
	BVH8::AlignedNode* n8 = nullptr;
	std::queue<BVH4::NodeRef> nodeQueue4;
	std::queue<BVH8::NodeRef> nodeQueue8;
	std::queue<unsigned long long> idQueue;

	std::queue<BBox3fa> boundsQueue;		// for leaf bounds

	std::ofstream f("bvh.txt");
	std::ofstream p("primitives.txt");
	/* if the scene contains only triangles, the BVH4 acceleration structure can be obtained this way */
	//=================================================BVH4 ONLY==========================================
	if (f.is_open()) {
		AccelData* accel = ((Accel*)scene)->intersectors.ptr;
		if (accel->type == AccelData::TY_BVH4) {
			bvh4 = (BVH4*)accel;
			BVH4::NodeRef node = bvh4->root;
			//std::cout << "TY_BVH4:\n";

			// initialize queue with root node and root id
			nodeQueue4.push(node);
			idQueue.push(0);
			while (!(nodeQueue4.empty())) {
				BVH4::NodeRef tempNode = nodeQueue4.front();
				nodeQueue4.pop();
				const unsigned long long tempID = idQueue.front();
				idQueue.pop();
				//std::cout << "\tNodeID: ";
				//std::cout << tempNode.type() << std::endl;

				
				if (tempNode.type() == 9 || tempNode.type() == 10) {	//leaf node
					size_t num;
					BBox3fa b;
					Triangle4v* tri = (Triangle4v*)tempNode.leaf(num);
					Triangle4* tri2 = (Triangle4*)tempNode.leaf(num);

					b = boundsQueue.front();
					boundsQueue.pop();

					// change primID to equal leaf node id
					if (scene->geometries[0]->gtype == 16) {		// GTY_TRIANGLE_MESH
						Triangle4v* tri = (Triangle4v*)tempNode.leaf(num);
						Triangle4* tri2 = (Triangle4*)tempNode.leaf(num);

						//tri[0].primIDs[0] = tempID;
						//std::cout << tri[0].primIDs[0] << std::endl;


						// write leaf node to bvh.txt
						f << tempID << " 2 " << tri->size() << " " << b.lower.x << " " << b.lower.y << " " << b.lower.z << " "
							<< b.upper.x << " " << b.upper.y << " " << b.upper.z << "\n";

						// write primitives to primitives.txt
						for (size_t i = 0; i < num; i++) {
							for (size_t j = 0; j < tri2[i].size(); j++) {
								p << tempID << " 1 " << tri2[i].geomID(j) << " " << tri2[i].primID(j) <<
									" " << tri2[i].v0.x[j] << " " << tri2[i].v0.y[j] << " " << tri2[i].v0.z[j] <<
									" " << tri2[i].v0.x[j] - tri2[i].e1.x[j] << " " << tri2[i].v0.y[j] - tri2[i].e1.y[j] << " " << tri2[i].v0.z[j] - tri2[i].e1.z[j] <<
									" " << tri2[i].v0.x[j] + tri2[i].e2.x[j] << " " << tri2[i].v0.y[j] + tri2[i].e2.y[j] << " " << tri2[i].v0.z[j] + tri2[i].e2.z[j] << "\n";
							}
						}
					}
					else if (scene->geometries[0]->gtype == 4) {	// GTY_FLAT_BEZIER_CURVE
						//const PrimRef& prim = (PrimRef)tempNode.leaf(num);
						Curve4v* curve = (Curve4v*)tempNode.leaf(num);
						BezierCurve3fa* bezCurve = (BezierCurve3fa*)tempNode.leaf(num);
						QuadraticBezierCurve3fa* quadBezCurve = (QuadraticBezierCurve3fa*)tempNode.leaf(num);

						//std::cout << "\t\t\tGeomID: " << curve->geomID(0) << ", PrimID: " << curve->primID << ", Bounds: ";
						//prim->bounds() << "\n";
					}
					else {
						std::cout << "UNKNOWN GEOM TYPE\n";
					}

				}
				//aligned node (inner node)
				else if (tempNode.type() == 0) {
					n4 = tempNode.alignedNode();
					vector <float> lowerX;
					vector <float> lowerY;
					vector <float> lowerZ;
					vector <float> upperX;
					vector <float> upperY;
					vector <float> upperZ;
					unsigned int numChildren = 0;
					for (int i = 0; i < 4; i++) {
						if (n4->bounds(i).lower.x == (float)pos_inf || n4->bounds(i).lower.x == (float)neg_inf) break;
						lowerX.push_back(n4->bounds(i).lower.x);
						lowerY.push_back(n4->bounds(i).lower.y);
						lowerZ.push_back(n4->bounds(i).lower.z);
						upperX.push_back(n4->bounds(i).upper.x);
						upperY.push_back(n4->bounds(i).upper.y);
						upperZ.push_back(n4->bounds(i).upper.z);
						numChildren++;
					}

					// write to txt file
					f << tempID << " 1 " << numChildren << " " << *std::min_element(lowerX.begin(), lowerX.end()) << " "
						<< *std::min_element(lowerY.begin(), lowerY.end()) << " " << *std::min_element(lowerZ.begin(), lowerZ.end()) << " "
						<< *std::max_element(upperX.begin(), upperX.end()) << " " << *std::max_element(upperY.begin(), upperY.end()) << " "
						<< *std::max_element(upperZ.begin(), upperZ.end()) << "\n";

					// push children to queue
					for (int i = 0; i < 4; i++) {
						nodeQueue4.push(n4->child(i));
						idQueue.push(tempID * 4 + (i + 1));

						// push leaf node bounds onto queue to access when current node is leaf node
						if (n4->child(i).type() == 9 || n4->child(i).type() == 10) {	// valid leaf node
							boundsQueue.push(n4->bounds(i));
						}
					}
				}
				else if (tempNode.type() == 8) {
					std::cout << "\tEMPTY NODE" << "\n";
				}
				else {
					std::cout << "ERROR: UNKNOWN NODE TYPE " << tempNode.type() << "\n";
				}
			}
		}
		//===================================BVH8 ONLY=============================
		else if (accel->type == AccelData::TY_BVH8) {
			bvh8 = (BVH8*)accel;
			BVH8::NodeRef node = bvh8->root;
			//std::cout << "TY_BVH8:\n";

			// initialize queue with root node and root id
			nodeQueue8.push(node);
			idQueue.push(0);
			while (!(nodeQueue8.empty())) {
				BVH8::NodeRef tempNode = nodeQueue8.front();
				nodeQueue8.pop();
				const unsigned long long tempID = idQueue.front();
				idQueue.pop();
				//std::cout << "\tNodeID: ";

				if (tempNode.type() == 9 || tempNode.type() == 10) {	//leaf node
					size_t num;
					BBox3fa b;
					Triangle4v* tri = (Triangle4v*)tempNode.leaf(num);
					Triangle4* tri2 = (Triangle4*)tempNode.leaf(num);

					b = boundsQueue.front();
					boundsQueue.pop();

					// change primID to equal leaf node id
					if (scene->geometries[0]->gtype == 16) {		// GTY_TRIANGLE_MESH
						Triangle4v* tri = (Triangle4v*)tempNode.leaf(num);
						Triangle4* tri2 = (Triangle4*)tempNode.leaf(num);

						//tri[0].primIDs[0] = tempID;
						//std::cout << tri[0].primIDs[0] << std::endl;


						// write leaf node to bvh.txt
						f << tempID << " 2 " << tri->size() << " " << b.lower.x << " " << b.lower.y << " " << b.lower.z << " "
							<< b.upper.x << " " << b.upper.y << " " << b.upper.z << "\n";

						// write primitives to primitives.txt
						for (size_t i = 0; i < num; i++) {
							for (size_t j = 0; j < tri2[i].size(); j++) {
								p << tempID << " 1 " << tri2[i].geomID(j) << " " << tri2[i].primID(j) <<
									" " << tri2[i].v0.x[j] << " " << tri2[i].v0.y[j] << " " << tri2[i].v0.z[j] <<
									" " << tri2[i].v0.x[j] - tri2[i].e1.x[j] << " " << tri2[i].v0.y[j] - tri2[i].e1.y[j] << " " << tri2[i].v0.z[j] - tri2[i].e1.z[j] <<
									" " << tri2[i].v0.x[j] + tri2[i].e2.x[j] << " " << tri2[i].v0.y[j] + tri2[i].e2.y[j] << " " << tri2[i].v0.z[j] + tri2[i].e2.z[j] << "\n";
							}
						}
					}
					else if (scene->geometries[0]->gtype == 4) {	// GTY_FLAT_BEZIER_CURVE
						//const PrimRef& prim = (PrimRef)tempNode.leaf(num);
						Curve4v* curve = (Curve4v*)tempNode.leaf(num);
						BezierCurve3fa* bezCurve = (BezierCurve3fa*)tempNode.leaf(num);
						QuadraticBezierCurve3fa* quadBezCurve = (QuadraticBezierCurve3fa*)tempNode.leaf(num);

						//std::cout << "\t\t\tGeomID: " << curve->geomID(0) << ", PrimID: " << curve->primID << ", Bounds: ";
						//prim->bounds() << "\n";
					}
					else {
						std::cout << "UNKNOWN GEOM TYPE\n";
					}
				}
				else if (tempNode.isAlignedNode()) {	//aligned node (inner node)
					n8 = tempNode.alignedNode();
					vector <float> lowerX;
					vector <float> lowerY;
					vector <float> lowerZ;
					vector <float> upperX;
					vector <float> upperY;
					vector <float> upperZ;
					unsigned int numChildren = 0;
					for (int i = 0; i < 8; i++) {
						if (n8->bounds(i).lower.x == (float)pos_inf || n8->bounds(i).lower.x == (float)neg_inf) break;
						lowerX.push_back(n8->bounds(i).lower.x);
						lowerY.push_back(n8->bounds(i).lower.y);
						lowerZ.push_back(n8->bounds(i).lower.z);
						upperX.push_back(n8->bounds(i).upper.x);
						upperY.push_back(n8->bounds(i).upper.y);
						upperZ.push_back(n8->bounds(i).upper.z);
						numChildren++;
					}

					// write to txt file
					f << tempID << " 1 " << numChildren << " " << *std::min_element(lowerX.begin(), lowerX.end()) << " "
						<< *std::min_element(lowerY.begin(), lowerY.end()) << " " << *std::min_element(lowerZ.begin(), lowerZ.end()) << " "
						<< *std::max_element(upperX.begin(), upperX.end()) << " " << *std::max_element(upperY.begin(), upperY.end()) << " "
						<< *std::max_element(upperZ.begin(), upperZ.end()) << "\n";

					// push children to queue
					for (int i = 0; i < 8; i++) {
						nodeQueue8.push(n8->child(i));
						idQueue.push(tempID * 8 + (i + 1));

						// push leaf node bounds onto queue to access when current node is leaf node
						if (n8->child(i).type() == 9) {	// valid leaf node
							boundsQueue.push(n8->bounds(i));
						}
					}
				}
				else if (tempNode.type() == 8) {
					std::cout << "\tEMPTY NODE" << "\n";
				}
				else {
					std::cout << "ERROR: UNKNOWN NODE TYPE " << tempNode.type() << "\n";
				}
			}
		}
		/* if there are also other geometry types, one has to iterate over the toplevel AccelN structure */
		//============================================= BOTH ==============================================================
		else if (accel->type == AccelData::TY_ACCELN)
		{
			/*
			std::cout << scene->geometries[0]->quality << "\n";
			std::cout << scene->geometries[1]->quality << "\n";
			std::cout << scene->geometries[2]->quality << "\n";
			std::cout << scene->geometries[0]->geomID << "\n";
			std::cout << scene->geometries[1]->geomID << "\n";
			std::cout << scene->geometries[2]->geomID << "\n";

			AccelN* test = ((AccelN*)scene->geometries[1]->scene->intersectors.ptr);
			*/
			AccelN* accelN = (AccelN*)(accel);
			std::cout << "Top level BVH consists of: \n";
			for (size_t i = 0; i < accelN->validAccels.size(); i++) {
				if (accelN->validAccels[i]->intersectors.ptr->type == AccelData::TY_BVH4) {
					bvh4 = (BVH4*)accelN->validAccels[i]->intersectors.ptr;
					BVH4::NodeRef node = bvh4->root;
					std::cout << "\tTY_BVH4:\n";

					// access instance associated with current bvh
					Instance* inst = ((Instance*)bvh4->scene->geometries[0].ptr);

					//std::cout << "\tLocal to World Transformation Matrix: \n";
					//std::cout << "\t\tvx = " << inst->local2world->l.vx << "\n";
					//std::cout << "\t\tvy = " << inst->local2world->l.vy << "\n";
					//std::cout << "\t\tvz = " << inst->local2world->l.vz << "\n";


					// initialize queue with root node and root id
					nodeQueue4.push(node);
					idQueue.push(0);
					while (!(nodeQueue4.empty())) {
						BVH4::NodeRef tempNode = nodeQueue4.front();
						nodeQueue4.pop();
						const unsigned long long tempID = idQueue.front();
						idQueue.pop();
						std::cout << "\t\tNodeID: ";
						//leaf node
						if (tempNode.isLeaf()) {
							std::cout << tempID << " Leaf Node \n";

							//BVH4::AlignedNode templeaf = tempNode.leaf.alignedNode();

						}
						//aligned node (inner node)
						else if (tempNode.isAlignedNode()) {
							n4 = tempNode.alignedNode();
							std::cout << tempID << ", Node Type: Aligned Node";
							for (int i = 0; i < 4; i++) {
								nodeQueue4.push(n4->child(i));
								idQueue.push(tempID * 4 + (i + 1));
								//std::cout << n4->bounds(i).lower.x << " " << n4->bounds(i).lower.y << " " << n4->bounds(i).lower.z << " " << n4->bounds(i).upper.x << " " << n4->bounds(i).upper.y << " " << n4->bounds(i).upper.z << " ";
								std::cout << "\n\t\t\tChild " << i << " bounds: " << n4->bounds(i) << ", NodeID: " << tempID * 4 + (i + 1);
							}
							std::cout << "\n";
						}
						//leaf node
						//something else
						else {
							std::cout << tempID << "ERROR: UNKNOWN NODE TYPE " << tempNode.type() << "\n";
						}
					}
				}
				else if (accelN->validAccels[i]->intersectors.ptr->type == AccelData::TY_BVH8) {
					bvh8 = (BVH8*)accelN->validAccels[i]->intersectors.ptr;
					BVH8::NodeRef node = bvh8->root;

					std::cout << "\tTY_BVH8:\n";

					// access instance associated with current bvh
					Instance* inst = ((Instance*)bvh8->scene->geometries[0].ptr);

					//std::cout << "\tLocal to World Transformation Matrix: \n";
					//std::cout << "\t\tvx = " << inst->local2world->l.vx << "\n";
					//std::cout << "\t\tvy = " << inst->local2world->l.vy << "\n";
					//std::cout << "\t\tvz = " << inst->local2world->l.vz << "\n";

					// initialize queue with root node and root id
					nodeQueue8.push(node);
					idQueue.push(0);
					while (!(nodeQueue8.empty())) {
						BVH8::NodeRef tempNode = nodeQueue8.front();
						nodeQueue8.pop();
						const unsigned long long tempID = idQueue.front();
						idQueue.pop();
						int nodeType = tempNode.type();
						std::cout << "\t\tNodeID: ";
						//leaf node
						if (tempNode.isLeaf()) {
							std::cout << tempID << " Leaf Node \n";
						}
						//aligned node (inner node)
						else if (tempNode.isAlignedNode()) {
							n8 = tempNode.alignedNode();
							std::cout << tempID << ", Node Type: Aligned Node";
							for (int i = 0; i < 8; i++) {
								nodeQueue8.push(n8->child(i));
								idQueue.push(tempID * 8 + (i + 1));
								//std::cout << n8->bounds(i).lower.x << " " << n8->bounds(i).lower.y << " " << n8->bounds(i).lower.z << " " << n8->bounds(i).upper.x << " " << n8->bounds(i).upper.y << " " << n8->bounds(i).upper.z << " ";
								std::cout << "\n\t\t\tChild " << i << " bounds: " << n8->bounds(i) << ", NodeID: " << tempID * 8 + (i + 1);
							}
							std::cout << "\n";
						}
						//something else
						else
							std::cout << tempID << "ERROR: UNKNOWN NODE TYPE " << tempNode.type() << "\n";
					}
				}
			}
		}
	}
	else {
		std::cout << "ERROR: UNABLE TO OPEN TEXT FILE";
	}
	f.close();
	p.close();
#endif // PRINT_CONSOLE
#endif // SKIP


    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcJoinCommitScene (RTCScene hscene) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcJoinCommitScene);
    RTC_VERIFY_HANDLE(hscene);
    scene->commit(true);
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcGetSceneBounds(RTCScene hscene, RTCBounds* bounds_o)
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetSceneBounds);
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    BBox3fa bounds = scene->bounds.bounds();
    bounds_o->lower_x = bounds.lower.x;
    bounds_o->lower_y = bounds.lower.y;
    bounds_o->lower_z = bounds.lower.z;
    bounds_o->align0  = 0;
    bounds_o->upper_x = bounds.upper.x;
    bounds_o->upper_y = bounds.upper.y;
    bounds_o->upper_z = bounds.upper.z;
    bounds_o->align1  = 0;
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcGetSceneLinearBounds(RTCScene hscene, RTCLinearBounds* bounds_o)
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetSceneBounds);
    RTC_VERIFY_HANDLE(hscene);
    if (bounds_o == nullptr)
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"invalid destination pointer");
    if (scene->isModified())
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    
    bounds_o->bounds0.lower_x = scene->bounds.bounds0.lower.x;
    bounds_o->bounds0.lower_y = scene->bounds.bounds0.lower.y;
    bounds_o->bounds0.lower_z = scene->bounds.bounds0.lower.z;
    bounds_o->bounds0.align0  = 0;
    bounds_o->bounds0.upper_x = scene->bounds.bounds0.upper.x;
    bounds_o->bounds0.upper_y = scene->bounds.bounds0.upper.y;
    bounds_o->bounds0.upper_z = scene->bounds.bounds0.upper.z;
    bounds_o->bounds0.align1  = 0;
    bounds_o->bounds1.lower_x = scene->bounds.bounds1.lower.x;
    bounds_o->bounds1.lower_y = scene->bounds.bounds1.lower.y;
    bounds_o->bounds1.lower_z = scene->bounds.bounds1.lower.z;
    bounds_o->bounds1.align0  = 0;
    bounds_o->bounds1.upper_x = scene->bounds.bounds1.upper.x;
    bounds_o->bounds1.upper_y = scene->bounds.bounds1.upper.y;
    bounds_o->bounds1.upper_z = scene->bounds.bounds1.upper.z;
    bounds_o->bounds1.align1  = 0;
    RTC_CATCH_END2(scene);
  }
  
  RTC_API void rtcIntersect1 (RTCScene hscene, RTCIntersectContext* user_context, RTCRayHit* rayhit) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcIntersect1);
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rayhit) & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT3(normal.travs,1,1,1);
    IntersectContext context(scene,user_context);
    scene->intersectors.intersect(*rayhit,&context);
#if defined(DEBUG)
    ((RayHit*)rayhit)->verifyHit();
#endif
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcIntersect4 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRayHit4* rayhit) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcIntersect4);

#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)rayhit)   & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,cnt,cnt,cnt);

    IntersectContext context(scene,user_context);
#if !defined(EMBREE_RAY_PACKETS)
    Ray4* ray4 = (Ray4*) rayhit;
    for (size_t i=0; i<4; i++) {
      if (!valid[i]) continue;
      RayHit ray1; ray4->get(i,ray1);
      scene->intersectors.intersect((RTCRayHit&)ray1,&context);
      ray4->set(i,ray1);
    }
#else
    scene->intersectors.intersect4(valid,*rayhit,&context);
#endif
    
    RTC_CATCH_END2(scene);
  }
  
  RTC_API void rtcIntersect8 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRayHit8* rayhit) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcIntersect8);

#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)rayhit)   & 0x1F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,cnt,cnt,cnt);

    IntersectContext context(scene,user_context);
#if !defined(EMBREE_RAY_PACKETS)
    Ray8* ray8 = (Ray8*) rayhit;
    for (size_t i=0; i<8; i++) {
      if (!valid[i]) continue;
      RayHit ray1; ray8->get(i,ray1);
      scene->intersectors.intersect((RTCRayHit&)ray1,&context);
      ray8->set(i,ray1);
    }
#else
    if (likely(scene->intersectors.intersector8))
      scene->intersectors.intersect8(valid,*rayhit,&context);
    else
      scene->device->rayStreamFilters.intersectSOA(scene,(char*)rayhit,8,1,sizeof(RTCRayHit8),&context);
#endif
    RTC_CATCH_END2(scene);
  }
  
  RTC_API void rtcIntersect16 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRayHit16* rayhit) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcIntersect16);

#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)rayhit)   & 0x3F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,cnt,cnt,cnt);

    IntersectContext context(scene,user_context);
#if !defined(EMBREE_RAY_PACKETS)
    Ray16* ray16 = (Ray16*) rayhit;
    for (size_t i=0; i<16; i++) {
      if (!valid[i]) continue;
      RayHit ray1; ray16->get(i,ray1);
      scene->intersectors.intersect((RTCRayHit&)ray1,&context);
      ray16->set(i,ray1);
    }
#else
    if (likely(scene->intersectors.intersector16))
      scene->intersectors.intersect16(valid,*rayhit,&context);
    else
      scene->device->rayStreamFilters.intersectSOA(scene,(char*)rayhit,16,1,sizeof(RTCRayHit16),&context);
#endif
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcIntersect1M (RTCScene hscene, RTCIntersectContext* user_context, RTCRayHit* rayhit, unsigned int M, size_t byteStride) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcIntersect1M);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rayhit ) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,M,M,M);
    IntersectContext context(scene,user_context);

    /* fast codepath for single rays */
    if (likely(M == 1)) {
      if (likely(rayhit->ray.tnear <= rayhit->ray.tfar)) 
        scene->intersectors.intersect(*rayhit,&context);
    } 

    /* codepath for streams */
    else {
      scene->device->rayStreamFilters.intersectAOS(scene,rayhit,M,byteStride,&context);   
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersect1M not supported");
#endif
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcIntersect1Mp (RTCScene hscene, RTCIntersectContext* user_context, RTCRayHit** rn, unsigned int M) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcIntersect1Mp);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rn) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,M,M,M);
    IntersectContext context(scene,user_context);

    /* fast codepath for single rays */
    if (likely(M == 1)) {
      if (likely(rn[0]->ray.tnear <= rn[0]->ray.tfar)) 
        scene->intersectors.intersect(*rn[0],&context);
    } 

    /* codepath for streams */
    else {
      scene->device->rayStreamFilters.intersectAOP(scene,rn,M,&context);
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersect1Mp not supported");
#endif
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcIntersectNM (RTCScene hscene, RTCIntersectContext* user_context, struct RTCRayHitN* rayhit, unsigned int N, unsigned int M, size_t byteStride) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcIntersectNM);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rayhit) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,N*M,N*M,N*M);
    IntersectContext context(scene,user_context);

    /* code path for single ray streams */
    if (likely(N == 1))
    {
      /* fast code path for streams of size 1 */
      if (likely(M == 1)) {
        if (likely(((RTCRayHit*)rayhit)->ray.tnear <= ((RTCRayHit*)rayhit)->ray.tfar))
          scene->intersectors.intersect(*(RTCRayHit*)rayhit,&context);
      } 
      /* normal codepath for single ray streams */
      else {
        scene->device->rayStreamFilters.intersectAOS(scene,(RTCRayHit*)rayhit,M,byteStride,&context);
      }
    }
    /* code path for ray packet streams */
    else {
      scene->device->rayStreamFilters.intersectSOA(scene,(char*)rayhit,N,M,byteStride,&context);
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersectNM not supported");
#endif
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcIntersectNp (RTCScene hscene, RTCIntersectContext* user_context, const RTCRayHitNp* rayhit, unsigned int N) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcIntersectNp);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rayhit->ray.org_x ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->ray.org_x not aligned to 4 bytes");   
    if (((size_t)rayhit->ray.org_y ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->ray.org_y not aligned to 4 bytes");   
    if (((size_t)rayhit->ray.org_z ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->ray.org_z not aligned to 4 bytes");   
    if (((size_t)rayhit->ray.dir_x ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->ray.dir_x not aligned to 4 bytes");   
    if (((size_t)rayhit->ray.dir_y ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->ray.dir_y not aligned to 4 bytes");   
    if (((size_t)rayhit->ray.dir_z ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->ray.dir_z not aligned to 4 bytes");   
    if (((size_t)rayhit->ray.tnear ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->ray.dir_x not aligned to 4 bytes");   
    if (((size_t)rayhit->ray.tfar  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->ray.tnear not aligned to 4 bytes");   
    if (((size_t)rayhit->ray.time  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->ray.time not aligned to 4 bytes");   
    if (((size_t)rayhit->ray.mask  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->ray.mask not aligned to 4 bytes");   
    if (((size_t)rayhit->hit.Ng_x  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->hit.Ng_x not aligned to 4 bytes");   
    if (((size_t)rayhit->hit.Ng_y  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->hit.Ng_y not aligned to 4 bytes");   
    if (((size_t)rayhit->hit.Ng_z  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->hit.Ng_z not aligned to 4 bytes");   
    if (((size_t)rayhit->hit.u     ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->hit.u not aligned to 4 bytes");   
    if (((size_t)rayhit->hit.v     ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->hit.v not aligned to 4 bytes");   
    if (((size_t)rayhit->hit.geomID) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->hit.geomID not aligned to 4 bytes");   
    if (((size_t)rayhit->hit.primID) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->hit.primID not aligned to 4 bytes");   
    if (((size_t)rayhit->hit.instID) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rayhit->hit.instID not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,N,N,N);
    IntersectContext context(scene,user_context);
    scene->device->rayStreamFilters.intersectSOP(scene,rayhit,N,&context);
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersectNp not supported");
#endif
    RTC_CATCH_END2(scene);
  }
  
  RTC_API void rtcOccluded1 (RTCScene hscene, RTCIntersectContext* user_context, RTCRay* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcOccluded1);
    STAT3(shadow.travs,1,1,1);
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)ray) & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    IntersectContext context(scene,user_context);
    scene->intersectors.occluded(*ray,&context);
    RTC_CATCH_END2(scene);
  }
  
  RTC_API void rtcOccluded4 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRay4* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcOccluded4);

#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)ray)   & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,cnt,cnt,cnt);

    IntersectContext context(scene,user_context);
#if !defined(EMBREE_RAY_PACKETS)
    RayHit4* ray4 = (RayHit4*) ray;
    for (size_t i=0; i<4; i++) {
      if (!valid[i]) continue;
      RayHit ray1; ray4->get(i,ray1);
      scene->intersectors.occluded((RTCRay&)ray1,&context);
      ray4->geomID[i] = ray1.geomID; 
    }
#else
    scene->intersectors.occluded4(valid,*ray,&context);
#endif
    
    RTC_CATCH_END2(scene);
  }
 
  RTC_API void rtcOccluded8 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRay8* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcOccluded8);

#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)ray)   & 0x1F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,cnt,cnt,cnt);

    IntersectContext context(scene,user_context);
#if !defined(EMBREE_RAY_PACKETS)
    RayHit8* ray8 = (RayHit8*) ray;
    for (size_t i=0; i<8; i++) {
      if (!valid[i]) continue;
      RayHit ray1; ray8->get(i,ray1);
      scene->intersectors.occluded((RTCRay&)ray1,&context);
      ray8->set(i,ray1);
    }
#else
    if (likely(scene->intersectors.intersector8))
      scene->intersectors.occluded8(valid,*ray,&context);
    else
      scene->device->rayStreamFilters.occludedSOA(scene,(char*)ray,8,1,sizeof(RTCRay8),&context);
#endif

    RTC_CATCH_END2(scene);
  }
  
  RTC_API void rtcOccluded16 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRay16* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcOccluded16);

#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)ray)   & 0x3F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,cnt,cnt,cnt);

    IntersectContext context(scene,user_context);
#if !defined(EMBREE_RAY_PACKETS)
    RayHit16* ray16 = (RayHit16*) ray;
    for (size_t i=0; i<16; i++) {
      if (!valid[i]) continue;
      RayHit ray1; ray16->get(i,ray1);
      scene->intersectors.occluded((RTCRay&)ray1,&context);
      ray16->set(i,ray1);
    }
#else
    if (likely(scene->intersectors.intersector16))
      scene->intersectors.occluded16(valid,*ray,&context);
    else
      scene->device->rayStreamFilters.occludedSOA(scene,(char*)ray,16,1,sizeof(RTCRay16),&context);
#endif

    RTC_CATCH_END2(scene);
  }
  
  RTC_API void rtcOccluded1M(RTCScene hscene, RTCIntersectContext* user_context, RTCRay* ray, unsigned int M, size_t byteStride) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcOccluded1M);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)ray) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,M,M,M);
    IntersectContext context(scene,user_context);
    /* fast codepath for streams of size 1 */
    if (likely(M == 1)) {
      if (likely(ray->tnear <= ray->tfar)) 
        scene->intersectors.occluded (*ray,&context);
    } 
    /* codepath for normal streams */
    else {
      scene->device->rayStreamFilters.occludedAOS(scene,ray,M,byteStride,&context);
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcOccluded1M not supported");
#endif
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcOccluded1Mp(RTCScene hscene, RTCIntersectContext* user_context, RTCRay** ray, unsigned int M) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcOccluded1Mp);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)ray) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,M,M,M);
    IntersectContext context(scene,user_context);

    /* fast codepath for streams of size 1 */
    if (likely(M == 1)) {
      if (likely(ray[0]->tnear <= ray[0]->tfar)) 
        scene->intersectors.occluded (*ray[0],&context);
    } 
    /* codepath for normal streams */
    else {
      scene->device->rayStreamFilters.occludedAOP(scene,ray,M,&context);
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcOccluded1Mp not supported");
#endif
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcOccludedNM(RTCScene hscene, RTCIntersectContext* user_context, RTCRayN* ray, unsigned int N, unsigned int M, size_t byteStride)
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcOccludedNM);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (byteStride < sizeof(RTCRayHit)) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"byteStride too small");
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)ray) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,N*M,N*N,N*N);
    IntersectContext context(scene,user_context);

    /* codepath for single rays */
    if (likely(N == 1))
    {
      /* fast path for streams of size 1 */
      if (likely(M == 1)) {
        if (likely(((RTCRay*)ray)->tnear <= ((RTCRay*)ray)->tfar))
          scene->intersectors.occluded (*(RTCRay*)ray,&context);
      } 
      /* codepath for normal ray streams */
      else {
        scene->device->rayStreamFilters.occludedAOS(scene,(RTCRay*)ray,M,byteStride,&context);
      }
    }
    /* code path for ray packet streams */
    else {
      scene->device->rayStreamFilters.occludedSOA(scene,(char*)ray,N,M,byteStride,&context);
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcOccludedNM not supported");
#endif
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcOccludedNp(RTCScene hscene, RTCIntersectContext* user_context, const RTCRayNp* ray, unsigned int N)
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcOccludedNp);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)ray->org_x ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "org_x not aligned to 4 bytes");   
    if (((size_t)ray->org_y ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "org_y not aligned to 4 bytes");   
    if (((size_t)ray->org_z ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "org_z not aligned to 4 bytes");   
    if (((size_t)ray->dir_x ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "dir_x not aligned to 4 bytes");   
    if (((size_t)ray->dir_y ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "dir_y not aligned to 4 bytes");   
    if (((size_t)ray->dir_z ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "dir_z not aligned to 4 bytes");   
    if (((size_t)ray->tnear ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "dir_x not aligned to 4 bytes");   
    if (((size_t)ray->tfar  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "tnear not aligned to 4 bytes");   
    if (((size_t)ray->time  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "time not aligned to 4 bytes");   
    if (((size_t)ray->mask  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,N,N,N);
    IntersectContext context(scene,user_context);
    scene->device->rayStreamFilters.occludedSOP(scene,ray,N,&context);
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcOccludedNp not supported");
#endif
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcRetainScene (RTCScene hscene) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcRetainScene);
    RTC_VERIFY_HANDLE(hscene);
    scene->refInc();
    RTC_CATCH_END2(scene);
  }
  
  RTC_API void rtcReleaseScene (RTCScene hscene) 
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcReleaseScene);
    RTC_VERIFY_HANDLE(hscene);
    scene->refDec();
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcSetGeometryInstancedScene(RTCGeometry hgeometry, RTCScene hscene)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    Ref<Scene> scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryInstancedScene);
    RTC_VERIFY_HANDLE(hgeometry);
    RTC_VERIFY_HANDLE(hscene);
    geometry->setInstancedScene(scene);
    RTC_CATCH_END2(geometry);
  }

  AffineSpace3fa loadTransform(RTCFormat format, const float* xfm)
  {
    AffineSpace3fa space = one;
    switch (format)
    {
    case RTC_FORMAT_FLOAT3X4_ROW_MAJOR:
      space = AffineSpace3fa(Vec3fa(xfm[ 0], xfm[ 4], xfm[ 8]),
                             Vec3fa(xfm[ 1], xfm[ 5], xfm[ 9]),
                             Vec3fa(xfm[ 2], xfm[ 6], xfm[10]),
                             Vec3fa(xfm[ 3], xfm[ 7], xfm[11]));
      break;

    case RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR:
      space = AffineSpace3fa(Vec3fa(xfm[ 0], xfm[ 1], xfm[ 2]),
                             Vec3fa(xfm[ 3], xfm[ 4], xfm[ 5]),
                             Vec3fa(xfm[ 6], xfm[ 7], xfm[ 8]),
                             Vec3fa(xfm[ 9], xfm[10], xfm[11]));
      break;

    case RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR:
      space = AffineSpace3fa(Vec3fa(xfm[ 0], xfm[ 1], xfm[ 2]),
                             Vec3fa(xfm[ 4], xfm[ 5], xfm[ 6]),
                             Vec3fa(xfm[ 8], xfm[ 9], xfm[10]),
                             Vec3fa(xfm[12], xfm[13], xfm[14]));
      break;

    default: 
      throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid matrix format");
      break;
    }
    return space;
  }

  void storeTransform(const AffineSpace3fa& space, RTCFormat format, float* xfm)
  {
    switch (format)
    {
    case RTC_FORMAT_FLOAT3X4_ROW_MAJOR:
      xfm[ 0] = space.l.vx.x;  xfm[ 1] = space.l.vy.x;  xfm[ 2] = space.l.vz.x;  xfm[ 3] = space.p.x;
      xfm[ 4] = space.l.vx.y;  xfm[ 5] = space.l.vy.y;  xfm[ 6] = space.l.vz.y;  xfm[ 7] = space.p.y;
      xfm[ 8] = space.l.vx.z;  xfm[ 9] = space.l.vy.z;  xfm[10] = space.l.vz.z;  xfm[11] = space.p.z;
      break;

    case RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR:
      xfm[ 0] = space.l.vx.x;  xfm[ 1] = space.l.vx.y;  xfm[ 2] = space.l.vx.z;
      xfm[ 3] = space.l.vy.x;  xfm[ 4] = space.l.vy.y;  xfm[ 5] = space.l.vy.z;
      xfm[ 6] = space.l.vz.x;  xfm[ 7] = space.l.vz.y;  xfm[ 8] = space.l.vz.z;
      xfm[ 9] = space.p.x;     xfm[10] = space.p.y;     xfm[11] = space.p.z;
      break;

    case RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR:
      xfm[ 0] = space.l.vx.x;  xfm[ 1] = space.l.vx.y;  xfm[ 2] = space.l.vx.z;  xfm[ 3] = 0.f;
      xfm[ 4] = space.l.vy.x;  xfm[ 5] = space.l.vy.y;  xfm[ 6] = space.l.vy.z;  xfm[ 7] = 0.f;
      xfm[ 8] = space.l.vz.x;  xfm[ 9] = space.l.vz.y;  xfm[10] = space.l.vz.z;  xfm[11] = 0.f;
      xfm[12] = space.p.x;     xfm[13] = space.p.y;     xfm[14] = space.p.z;     xfm[15] = 1.f;
      break;

    default:
      throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid matrix format");
      break;
    }
  }

  RTC_API void rtcSetGeometryTransform(RTCGeometry hgeometry, unsigned int timeStep, RTCFormat format, const void* xfm)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryTransform);
    RTC_VERIFY_HANDLE(hgeometry);
    RTC_VERIFY_HANDLE(xfm);
    const AffineSpace3fa transform = loadTransform(format, (const float*)xfm);
    geometry->setTransform(transform, timeStep);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcGetGeometryTransform(RTCGeometry hgeometry, float time, RTCFormat format, void* xfm)
  {
    Geometry* geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetGeometryTransform);
    const AffineSpace3fa transform = geometry->getTransform(time);
    storeTransform(transform, format, (float*)xfm);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcFilterIntersection(const struct RTCIntersectFunctionNArguments* const args_i, const struct RTCFilterFunctionNArguments* filter_args)
  {
    IntersectFunctionNArguments* args = (IntersectFunctionNArguments*) args_i;
    args->report(args,filter_args);
  }

  RTC_API void rtcFilterOcclusion(const struct RTCOccludedFunctionNArguments* const args_i, const struct RTCFilterFunctionNArguments* filter_args)
  {
    OccludedFunctionNArguments* args = (OccludedFunctionNArguments*) args_i;
    args->report(args,filter_args);
  }
  
  RTC_API RTCGeometry rtcNewGeometry (RTCDevice hdevice, RTCGeometryType type)
  {
    Device* device = (Device*) hdevice;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcNewGeometry);
    RTC_VERIFY_HANDLE(hdevice);

    switch (type)
    {
    case RTC_GEOMETRY_TYPE_TRIANGLE:
    {
#if defined(EMBREE_GEOMETRY_TRIANGLE)
      createTriangleMeshTy createTriangleMesh = nullptr;
      SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(device->enabled_cpu_features,createTriangleMesh);
      Geometry* geom = createTriangleMesh(device);
      return (RTCGeometry) geom->refInc();
#else
      throw_RTCError(RTC_ERROR_UNKNOWN,"RTC_GEOMETRY_TYPE_TRIANGLE is not supported");
#endif
    }
    
    case RTC_GEOMETRY_TYPE_QUAD:
    {
#if defined(EMBREE_GEOMETRY_QUAD)
      createQuadMeshTy createQuadMesh = nullptr;
      SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(device->enabled_cpu_features,createQuadMesh);
      Geometry* geom = createQuadMesh(device);
      return (RTCGeometry) geom->refInc();
#else
      throw_RTCError(RTC_ERROR_UNKNOWN,"RTC_GEOMETRY_TYPE_QUAD is not supported");
#endif
    }
    
    case RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE:
      
    case RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE:
    case RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE:
    case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE:
      
    case RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE:
    case RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE:
    case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE:

    case RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE:
    case RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE:
    case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE:
    {
#if defined(EMBREE_GEOMETRY_CURVE)
      createLineSegmentsTy createLineSegments = nullptr;
      SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(device->enabled_cpu_features,createLineSegments);
      createCurvesTy createCurves = nullptr;
      SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(device->enabled_cpu_features,createCurves);
      
      Geometry* geom;
      switch (type) {
      //case RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE            : geom = createLineSegments (device,Geometry::GTY_ROUND_LINEAR_CURVE); break;
      case RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE             : geom = createLineSegments (device,Geometry::GTY_FLAT_LINEAR_CURVE); break;
      //case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_LINEAR_CURVE  : geom = createLineSegments (device,Geometry::GTY_ORIENTED_LINEAR_CURVE); break;
        
      case RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE            : geom = createCurves(device,Geometry::GTY_ROUND_BEZIER_CURVE); break;
      case RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE             : geom = createCurves(device,Geometry::GTY_FLAT_BEZIER_CURVE); break;
      case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE  : geom = createCurves(device,Geometry::GTY_ORIENTED_BEZIER_CURVE); break;
        
      case RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE           : geom = createCurves(device,Geometry::GTY_ROUND_BSPLINE_CURVE); break;
      case RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE            : geom = createCurves(device,Geometry::GTY_FLAT_BSPLINE_CURVE); break;
      case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE : geom = createCurves(device,Geometry::GTY_ORIENTED_BSPLINE_CURVE); break;
        
      case RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE           : geom = createCurves(device,Geometry::GTY_ROUND_HERMITE_CURVE); break;
      case RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE            : geom = createCurves(device,Geometry::GTY_FLAT_HERMITE_CURVE); break;
      case RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE : geom = createCurves(device,Geometry::GTY_ORIENTED_HERMITE_CURVE); break;
      default:                                    geom = nullptr; break;
      }
      return (RTCGeometry) geom->refInc();
#else
      throw_RTCError(RTC_ERROR_UNKNOWN,"RTC_GEOMETRY_TYPE_CURVE is not supported");
#endif
    }
    
    case RTC_GEOMETRY_TYPE_SUBDIVISION:
    {
#if defined(EMBREE_GEOMETRY_SUBDIVISION)
      createSubdivMeshTy createSubdivMesh = nullptr;
      SELECT_SYMBOL_DEFAULT_AVX(device->enabled_cpu_features,createSubdivMesh);
      //SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(device->enabled_cpu_features,createSubdivMesh); // FIXME: this does not work for some reason?
      Geometry* geom = createSubdivMesh(device);
      return (RTCGeometry) geom->refInc();
#else
      throw_RTCError(RTC_ERROR_UNKNOWN,"RTC_GEOMETRY_TYPE_SUBDIVISION is not supported");
#endif
    }
    
    case RTC_GEOMETRY_TYPE_USER:
    {
#if defined(EMBREE_GEOMETRY_USER)
      createUserGeometryTy createUserGeometry = nullptr;
      SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(device->enabled_cpu_features,createUserGeometry);
      Geometry* geom = createUserGeometry(device);
      return (RTCGeometry) geom->refInc();
#else
      throw_RTCError(RTC_ERROR_UNKNOWN,"RTC_GEOMETRY_TYPE_USER is not supported");
#endif
    }

    case RTC_GEOMETRY_TYPE_INSTANCE:
    {
#if defined(EMBREE_GEOMETRY_USER)
      createInstanceTy createInstance = nullptr;
      SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(device->enabled_cpu_features,createInstance);
      Geometry* geom = createInstance(device);
      return (RTCGeometry) geom->refInc();
#else
      throw_RTCError(RTC_ERROR_UNKNOWN,"RTC_GEOMETRY_TYPE_INSTANCE is not supported");
#endif
    }

    case RTC_GEOMETRY_TYPE_GRID:
    {
#if defined(EMBREE_GEOMETRY_GRID)
      createGridMeshTy createGridMesh = nullptr;
      SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512KNL_AVX512SKX(device->enabled_cpu_features,createGridMesh);
      Geometry* geom = createGridMesh(device);
      return (RTCGeometry) geom->refInc();
#else
      throw_RTCError(RTC_ERROR_UNKNOWN,"RTC_GEOMETRY_TYPE_GRID is not supported");
#endif
    }
    
    default:
      throw_RTCError(RTC_ERROR_UNKNOWN,"invalid geometry type");
    }
    
    RTC_CATCH_END(device);
    return nullptr;
  }
  
  RTC_API void rtcSetGeometryUserPrimitiveCount(RTCGeometry hgeometry, unsigned int userPrimitiveCount)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryUserPrimitiveCount);
    RTC_VERIFY_HANDLE(hgeometry);
    
    if (unlikely(geometry->getType() != Geometry::GTY_USER_GEOMETRY))
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"operation only allowed for user geometries"); 

    geometry->setNumPrimitives(userPrimitiveCount);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometryTimeStepCount(RTCGeometry hgeometry, unsigned int timeStepCount)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryTimeStepCount);
    RTC_VERIFY_HANDLE(hgeometry);

    if (timeStepCount > RTC_MAX_TIME_STEP_COUNT)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"number of time steps is out of range");
    
    geometry->setNumTimeSteps(timeStepCount);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometryVertexAttributeCount(RTCGeometry hgeometry, unsigned int N)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryVertexAttributeCount);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setVertexAttributeCount(N);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometryTopologyCount(RTCGeometry hgeometry, unsigned int N)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryTopologyCount);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setTopologyCount(N);
    RTC_CATCH_END2(geometry);
  }
 
  /*! sets the build quality of the geometry */
  RTC_API void rtcSetGeometryBuildQuality (RTCGeometry hgeometry, RTCBuildQuality quality) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryBuildQuality);
    RTC_VERIFY_HANDLE(hgeometry);
    if (quality != RTC_BUILD_QUALITY_LOW &&
        quality != RTC_BUILD_QUALITY_MEDIUM &&
        quality != RTC_BUILD_QUALITY_HIGH &&
        quality != RTC_BUILD_QUALITY_REFIT)
      throw std::runtime_error("invalid build quality");
    geometry->setBuildQuality(quality);
    RTC_CATCH_END2(geometry);
  }
  
  RTC_API void rtcSetGeometryMask (RTCGeometry hgeometry, unsigned int mask) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryMask);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setMask(mask);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometrySubdivisionMode (RTCGeometry hgeometry, unsigned topologyID, RTCSubdivisionMode mode) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometrySubdivisionMode);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setSubdivisionMode(topologyID,mode);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometryVertexAttributeTopology(RTCGeometry hgeometry, unsigned int vertexAttributeID, unsigned int topologyID)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryVertexAttributeTopology);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setVertexAttributeTopology(vertexAttributeID, topologyID);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometryBuffer(RTCGeometry hgeometry, RTCBufferType type, unsigned int slot, RTCFormat format, RTCBuffer hbuffer, size_t byteOffset, size_t byteStride, size_t itemCount)
  {
    Ref<Geometry> geometry = (Geometry*)hgeometry;
    Ref<Buffer> buffer = (Buffer*)hbuffer;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryBuffer);
    RTC_VERIFY_HANDLE(hgeometry);
    RTC_VERIFY_HANDLE(hbuffer);
    
    if (geometry->device != buffer->device)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"inputs are from different devices");
    
    if (itemCount > 0xFFFFFFFFu)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"buffer too large");
    
    geometry->setBuffer(type, slot, format, buffer, byteOffset, byteStride, (unsigned int)itemCount);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetSharedGeometryBuffer(RTCGeometry hgeometry, RTCBufferType type, unsigned int slot, RTCFormat format, const void* ptr, size_t byteOffset, size_t byteStride, size_t itemCount)
  {
    Ref<Geometry> geometry = (Geometry*)hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetSharedGeometryBuffer);
    RTC_VERIFY_HANDLE(hgeometry);
    
    if (itemCount > 0xFFFFFFFFu)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"buffer too large");
    
    Ref<Buffer> buffer = new Buffer(geometry->device, itemCount*byteStride, (char*)ptr + byteOffset);
    geometry->setBuffer(type, slot, format, buffer, 0, byteStride, (unsigned int)itemCount);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void* rtcSetNewGeometryBuffer(RTCGeometry hgeometry, RTCBufferType type, unsigned int slot, RTCFormat format, size_t byteStride, size_t itemCount)
  {
    Ref<Geometry> geometry = (Geometry*)hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetNewGeometryBuffer);
    RTC_VERIFY_HANDLE(hgeometry);

    if (itemCount > 0xFFFFFFFFu)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"buffer too large");
    
    /* vertex buffers need to get overallocated slightly as elements are accessed using SSE loads */
    size_t bytes = itemCount*byteStride;
    if (type == RTC_BUFFER_TYPE_VERTEX || type == RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE)
      bytes += (byteStride+15)%16 - byteStride;
      
    Ref<Buffer> buffer = new Buffer(geometry->device, bytes);
    geometry->setBuffer(type, slot, format, buffer, 0, byteStride, (unsigned int)itemCount);
    return buffer->data();
    RTC_CATCH_END2(geometry);
    return nullptr;
  }

  RTC_API void* rtcGetGeometryBufferData(RTCGeometry hgeometry, RTCBufferType type, unsigned int slot)
  {
    Ref<Geometry> geometry = (Geometry*)hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetGeometryBufferData);
    RTC_VERIFY_HANDLE(hgeometry);
    return geometry->getBuffer(type, slot);
    RTC_CATCH_END2(geometry);
    return nullptr;
  }
  
  RTC_API void rtcEnableGeometry (RTCGeometry hgeometry) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcEnableGeometry);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->enable();
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcUpdateGeometryBuffer (RTCGeometry hgeometry, RTCBufferType type, unsigned int slot) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcUpdateGeometryBuffer);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->updateBuffer(type, slot);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcDisableGeometry (RTCGeometry hgeometry) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcDisableGeometry);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->disable();
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometryTessellationRate (RTCGeometry hgeometry, float tessellationRate)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryTessellationRate);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setTessellationRate(tessellationRate);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometryUserData (RTCGeometry hgeometry, void* ptr) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryUserData);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setUserData(ptr);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void* rtcGetGeometryUserData (RTCGeometry hgeometry)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetGeometryUserData);
    RTC_VERIFY_HANDLE(hgeometry);
    return geometry->getUserData();
    RTC_CATCH_END2(geometry);
    return nullptr;
  }

  RTC_API void rtcSetGeometryBoundsFunction (RTCGeometry hgeometry, RTCBoundsFunction bounds, void* userPtr)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryBoundsFunction);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setBoundsFunction(bounds,userPtr);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometryDisplacementFunction (RTCGeometry hgeometry, RTCDisplacementFunctionN displacement)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryDisplacementFunction);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setDisplacementFunction(displacement);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometryIntersectFunction (RTCGeometry hgeometry, RTCIntersectFunctionN intersect) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryIntersectFunction);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setIntersectFunctionN(intersect);
    RTC_CATCH_END2(geometry);
  }

  RTC_API unsigned int rtcGetGeometryFirstHalfEdge(RTCGeometry hgeometry, unsigned int faceID)
  {
    Geometry* geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetGeometryFirstHalfEdge);
    return geometry->getFirstHalfEdge(faceID);
    RTC_CATCH_END2(geometry);
    return -1;
  }

  RTC_API unsigned int rtcGetGeometryFace(RTCGeometry hgeometry, unsigned int edgeID)
  {
    Geometry* geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetGeometryFace);
    return geometry->getFace(edgeID);
    RTC_CATCH_END2(geometry);
    return -1;
  }

  RTC_API unsigned int rtcGetGeometryNextHalfEdge(RTCGeometry hgeometry, unsigned int edgeID)
  {
    Geometry* geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetGeometryNextHalfEdge);
    return geometry->getNextHalfEdge(edgeID);
    RTC_CATCH_END2(geometry);
    return -1;
  }

  RTC_API unsigned int rtcGetGeometryPreviousHalfEdge(RTCGeometry hgeometry, unsigned int edgeID)
  {
    Geometry* geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetGeometryPreviousHalfEdge);
    return geometry->getPreviousHalfEdge(edgeID);
    RTC_CATCH_END2(geometry);
    return -1;
  }

  RTC_API unsigned int rtcGetGeometryOppositeHalfEdge(RTCGeometry hgeometry, unsigned int topologyID, unsigned int edgeID)
  {
    Geometry* geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetGeometryOppositeHalfEdge);
    return geometry->getOppositeHalfEdge(topologyID,edgeID);
    RTC_CATCH_END2(geometry);
    return -1;
  }

  RTC_API void rtcSetGeometryOccludedFunction (RTCGeometry hgeometry, RTCOccludedFunctionN occluded) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetOccludedFunctionN);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setOccludedFunctionN(occluded);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometryIntersectFilterFunction (RTCGeometry hgeometry, RTCFilterFunctionN filter) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryIntersectFilterFunction);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setIntersectionFilterFunctionN(filter);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcSetGeometryOccludedFilterFunction (RTCGeometry hgeometry, RTCFilterFunctionN filter) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcSetGeometryOccludedFilterFunction);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->setOcclusionFilterFunctionN(filter);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcInterpolate(const RTCInterpolateArguments* const args)
  {
    Geometry* geometry = (Geometry*) args->geometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcInterpolate);
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(args->geometry);
#endif
    geometry->interpolate(args);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcInterpolateN(const RTCInterpolateNArguments* const args)
  {
    Geometry* geometry = (Geometry*) args->geometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcInterpolateN);
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(args->geometry);
#endif
    geometry->interpolateN(args);
    RTC_CATCH_END2(geometry);
  }

  RTC_API void rtcCommitGeometry (RTCGeometry hgeometry)
  {
    Geometry* geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcCommitGeometry);
    RTC_VERIFY_HANDLE(hgeometry);
    return geometry->commit();
    RTC_CATCH_END2(geometry);
  }

  RTC_API unsigned int rtcAttachGeometry (RTCScene hscene, RTCGeometry hgeometry)
  {
    Scene* scene = (Scene*) hscene;
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcAttachGeometry);
    RTC_VERIFY_HANDLE(hscene);
    RTC_VERIFY_HANDLE(hgeometry);
    if (scene->device != geometry->device)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"inputs are from different devices");
    return scene->bind(RTC_INVALID_GEOMETRY_ID,geometry);
    RTC_CATCH_END2(scene);
    return -1;
  }

  RTC_API void rtcAttachGeometryByID (RTCScene hscene, RTCGeometry hgeometry, unsigned int geomID)
  {
    Scene* scene = (Scene*) hscene;
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcAttachGeometryByID);
    RTC_VERIFY_HANDLE(hscene);
    RTC_VERIFY_HANDLE(hgeometry);
    RTC_VERIFY_GEOMID(geomID);
    if (scene->device != geometry->device)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"inputs are from different devices");
    scene->bind(geomID,geometry);
    RTC_CATCH_END2(scene);
  }
  
  RTC_API void rtcDetachGeometry (RTCScene hscene, unsigned int geomID)
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcDetachGeometry);
    RTC_VERIFY_HANDLE(hscene);
    RTC_VERIFY_GEOMID(geomID);
    scene->detachGeometry(geomID);
    RTC_CATCH_END2(scene);
  }

  RTC_API void rtcRetainGeometry (RTCGeometry hgeometry)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcRetainGeometry);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->refInc();
    RTC_CATCH_END2(geometry);
  }
  
  RTC_API void rtcReleaseGeometry (RTCGeometry hgeometry)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcReleaseGeometry);
    RTC_VERIFY_HANDLE(hgeometry);
    geometry->refDec();
    RTC_CATCH_END2(geometry);
  }

  RTC_API RTCGeometry rtcGetGeometry (RTCScene hscene, unsigned int geomID)
  {
    Scene* scene = (Scene*) hscene;
    RTC_CATCH_BEGIN;
    RTC_TRACE(rtcGetGeometry);
#if defined(DEBUG)
    RTC_VERIFY_HANDLE(hscene);
    RTC_VERIFY_GEOMID(geomID);
#endif
    return (RTCGeometry) scene->get(geomID);
    RTC_CATCH_END2(scene);
    return nullptr;
  }
}
