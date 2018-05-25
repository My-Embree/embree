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

#include "../common/tutorial/tutorial_device.h"
#include <map>
#include <iterator>
#include <queue>
#include <iostream>
#include <fstream>

//#define BUILD_MAP
#define BUILD_TXT

namespace embree
{	
  RTCScene g_scene  = nullptr;

  /* This function is called by the builder to signal progress and to
   * report memory consumption. */
  bool memoryMonitor(void* userPtr, ssize_t bytes, bool post) {
    return true;
  }

  bool buildProgress (void* userPtr, double f) {
    return true;
  }

  void splitPrimitive (const RTCBuildPrimitive* prim, unsigned int dim, float pos, RTCBounds* lprim, RTCBounds* rprim, void* userPtr)
  {
    assert(dim < 3);
    assert(prim->geomID == 0);
    *(BBox3fa*) lprim = *(BBox3fa*) prim;
    *(BBox3fa*) rprim = *(BBox3fa*) prim;
    (&lprim->upper_x)[dim] = pos;
    (&rprim->lower_x)[dim] = pos;
  }

  struct Node
  {
    virtual float sah() = 0;
	unsigned int nodeType;
  };

  struct InnerNode : public Node
  {
    BBox3fa bounds[2];
    Node* children[2];

    InnerNode() {
      bounds[0] = bounds[1] = empty;
      children[0] = children[1] = nullptr;
	  nodeType = 1;
    }

    float sah() {
      return 1.0f + (area(bounds[0])*children[0]->sah() + area(bounds[1])*children[1]->sah())/area(merge(bounds[0],bounds[1]));
    }

    static void* create (RTCThreadLocalAllocator alloc, unsigned int numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      void* ptr = rtcThreadLocalAlloc(alloc,sizeof(InnerNode),16);
      return (void*) new (ptr) InnerNode;
    }

    static void  setChildren (void* nodePtr, void** childPtr, unsigned int numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      for (size_t i=0; i<2; i++)
        ((InnerNode*)nodePtr)->children[i] = (Node*) childPtr[i];
    }

    static void  setBounds (void* nodePtr, const RTCBounds** bounds, unsigned int numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      for (size_t i=0; i<2; i++)
        ((InnerNode*)nodePtr)->bounds[i] = *(const BBox3fa*) bounds[i];
    }
  };

  struct LeafNode : public Node
  {
    unsigned id;
    BBox3fa bounds;

	LeafNode(unsigned id_new, const BBox3fa& bounds_new) {
		id = id_new;
		bounds = bounds_new;
		nodeType = 2;
	}

    float sah() {
      return 1.0f;
    }

    static void* create (RTCThreadLocalAllocator alloc, const RTCBuildPrimitive* prims, size_t numPrims, void* userPtr)
    {
      assert(numPrims == 1);
      void* ptr = rtcThreadLocalAlloc(alloc,sizeof(LeafNode),16);
      return (void*) new (ptr) LeafNode(prims->primID,*(BBox3fa*)prims);
    }
  };

  void buildTXT(InnerNode* root) {
	  std::queue<Node*> nodeQueue;
	  std::queue<int> idQueue;

	  // initialize queue with root node and root id
	  nodeQueue.push(root);
	  idQueue.push(0);

	  InnerNode* nodeTemp;
	  unsigned int idTemp;

	  std::ofstream f("nodes.txt");

	  if (f.is_open()) {
		  while (!nodeQueue.empty()) {
			  // pop current node
			  nodeTemp = (InnerNode*)nodeQueue.front();
			  nodeQueue.pop();

			  // pop current node id
			  idTemp = idQueue.front();
			  idQueue.pop();

			  // leaf nodes only point to primitives, check node type first
			  if (nodeTemp->nodeType == 1) {		// inner node
				  nodeQueue.push(((InnerNode*)nodeTemp)->children[0]);
				  idQueue.push(2 * idTemp + 1);

				  nodeQueue.push(((InnerNode*)nodeTemp)->children[1]);
				  idQueue.push(2 * idTemp + 2);
			  }
			  else {
				  //std::cout << "LEAF NODE POPPED" << std::endl;
			  }

			  // write to text file, each line in file is a node
			  f << idTemp << " " << nodeTemp->nodeType << " " << (unsigned int)nodeTemp->bounds[0].lower.x << " " << (unsigned int)nodeTemp->bounds[0].lower.y << " " <<
				  (unsigned int)nodeTemp->bounds[0].lower.z << " " << (unsigned int)nodeTemp->bounds[0].upper.x << " " << (unsigned int)nodeTemp->bounds[0].upper.y << " " <<
				  (unsigned int)nodeTemp->bounds[0].upper.z << " " << (unsigned int)nodeTemp->bounds[1].lower.x << " " << (unsigned int)nodeTemp->bounds[1].lower.y << " " <<
				  (unsigned int)nodeTemp->bounds[1].lower.z << " " << (unsigned int)nodeTemp->bounds[1].upper.x << " " << (unsigned int)nodeTemp->bounds[1].upper.y << " " <<
				  (unsigned int)nodeTemp->bounds[1].upper.z << "\n";

		  }

		  f.close();
	  }
	  else {
		  std::cout << "Unable to open file" << std::endl;
	  }
  }


  void buildMap(std::map<unsigned int, long long> &nodeMap, InnerNode* root) {
	std::queue<Node*> nodeQueue;
	std::queue<int> idQueue;

	// initialize queue with root node and root id
	nodeQueue.push(root);
	idQueue.push(0);

	Node* nodeTemp;
	int idTemp;

	while (!nodeQueue.empty()) {
		// pop current node
		nodeTemp = nodeQueue.front();
		nodeQueue.pop();

		// pop current node id
		idTemp = idQueue.front();
		idQueue.pop();

		// leaf nodes only point to primitives, check node type first
		if (nodeTemp->nodeType == 1) {		// inner node
			nodeQueue.push(((InnerNode*)nodeTemp)->children[0]);
			idQueue.push(2 * idTemp + 1);

			nodeQueue.push(((InnerNode*)nodeTemp)->children[1]);
			idQueue.push(2 * idTemp + 2);
		}
		else {
			//std::cout << "LEAF NODE POPPED" << std::endl;
		}

		// insert node id and node type to map
		nodeMap.insert(std::pair<unsigned int, long long>(idTemp, (long long)nodeTemp));
	}
  }

  // prints tree
  void printMap(std::map<int, long long> &nodeMap) {
	  
	  std::map<int, long long>::iterator itr;
	  std::cout << "\nThe map for the BVH is : \n";
	  std::cout << "\tKEY\tELEMENT\n";
	  for (itr = nodeMap.begin(); itr != nodeMap.end(); ++itr)
	  {
		  std::cout << '\t' << itr->first
			  << '\t' << itr->second << '\n';
		  /*
		  if (itr->second != 1) {
			  std::cout << "LEAF" << std::endl;
		  }
		  */
	  }
	  std::cout << std::endl;
  }
  

  void build(RTCBuildQuality quality, avector<RTCBuildPrimitive>& prims_i, char* cfg, size_t extraSpace = 0)
  {
    rtcSetDeviceMemoryMonitorFunction(g_device,memoryMonitor,nullptr);

    RTCBVH bvh = rtcNewBVH(g_device);

    avector<RTCBuildPrimitive> prims;
    prims.reserve(prims_i.size()+extraSpace);
    prims.resize(prims_i.size());

    /* settings for BVH build */
    RTCBuildArguments arguments = rtcDefaultBuildArguments();
    arguments.byteSize = sizeof(arguments);
    arguments.buildFlags = RTC_BUILD_FLAG_DYNAMIC;
    arguments.buildQuality = quality;
    arguments.maxBranchingFactor = 2;
    arguments.maxDepth = 1024;
    arguments.sahBlockSize = 1;
    arguments.minLeafSize = 1;
    arguments.maxLeafSize = 1;
    arguments.traversalCost = 1.0f;
    arguments.intersectionCost = 1.0f;
    arguments.bvh = bvh;
    arguments.primitives = prims.data();
    arguments.primitiveCount = prims.size();
    arguments.primitiveArrayCapacity = prims.capacity();
    arguments.createNode = InnerNode::create;
    arguments.setNodeChildren = InnerNode::setChildren;
    arguments.setNodeBounds = InnerNode::setBounds;
    arguments.createLeaf = LeafNode::create;
    arguments.splitPrimitive = splitPrimitive;
    arguments.buildProgress = buildProgress;
    arguments.userPtr = nullptr;
    

    /* we recreate the prims array here, as the builders modify this array */
    for (size_t j=0; j<prims.size(); j++) prims[j] = prims_i[j];

    std::cout << "Building BVH over " << prims.size() << " primitives, " << std::flush;
    double t0 = getSeconds();
    Node* root = (Node*) rtcBuildBVH(&arguments);
    double t1 = getSeconds();
    const float sah = root ? root->sah() : 0.0f;
    std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(prims.size())/(t1-t0) << " Mprims/s, sah = " << sah << " [DONE]" << std::endl;


#ifdef	BUILD_MAP
	std::map<unsigned int, long long> nodeMap;
	buildMap(nodeMap, (InnerNode*)root);
	printMap(nodeMap);
#endif // BUILD_MAP


#ifdef BUILD_TXT
	buildTXT((InnerNode*)root);
#endif // BUILD_TXT


    rtcReleaseBVH(bvh);
  }

  /* called by the C++ code for initialization */
  extern "C" void device_init (char* cfg)
  {
    /* set start render mode */
    renderTile = renderTileStandard;

    /* create random bounding boxes */
    //const size_t N = 2300000;
	const size_t N = 200;
    const size_t extraSpace = 100;
    avector<RTCBuildPrimitive> prims;
    prims.resize(N);

	/*	Create primitives	*/
    for (size_t i=0; i<N; i++)
    {
      const float x = float(drand48());
      const float y = float(drand48());
      const float z = float(drand48());
      const Vec3fa p = 1000.0f*Vec3fa(x,y,z);
      const BBox3fa b = BBox3fa(p,p+Vec3fa(1.0f));

      RTCBuildPrimitive prim;
      prim.lower_x = b.lower.x;
      prim.lower_y = b.lower.y;
      prim.lower_z = b.lower.z;
      prim.geomID = 0;
      prim.upper_x = b.upper.x;
      prim.upper_y = b.upper.y;
      prim.upper_z = b.upper.z;
      prim.primID = (unsigned) i;
      prims[i] = prim;
    }

	/*	only want to test high quality build
    std::cout << "Low quality BVH build:" << std::endl;
    build(RTC_BUILD_QUALITY_LOW,prims,cfg);

    std::cout << "Normal quality BVH build:" << std::endl;
    build(RTC_BUILD_QUALITY_MEDIUM,prims,cfg);
	*/
    std::cout << "High quality BVH build:" << std::endl;
    build(RTC_BUILD_QUALITY_HIGH,prims,cfg,extraSpace);
  }

  /* task that renders a single screen tile */
  void renderTileStandard(int taskIndex, int threadIndex, int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera,
                          const int numTilesX,
                          const int numTilesY)
  {
  }

  /* task that renders a single screen tile */
  void renderTileTask(int taskIndex, int threadIndex, int* pixels,
                      const unsigned int width,
                      const unsigned int height,
                      const float time,
                      const ISPCCamera& camera,
                      const int numTilesX,
                      const int numTilesY)
  {
  }

  /* called by the C++ code to render */
  extern "C" void device_render (int* pixels,
                                 const int width,
                                 const int height,
                                 const float time,
                                 const ISPCCamera& camera)
  {
  }

  /* called by the C++ code for cleanup */
  extern "C" void device_cleanup () {
  }
}
