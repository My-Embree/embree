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

#pragma once

#include "../common/ray.h"
#include "../common/geometry.h"

#include <fstream>
#include <iostream>
#include <bitset>

namespace embree
{
  namespace isa
  {
    struct CurvePrecalculations1
    {
      float depth_scale;
      LinearSpace3fa ray_space;
           
      __forceinline CurvePrecalculations1() {}

      __forceinline CurvePrecalculations1(const Ray& ray, const void* ptr)
      {
		// open file
		std::ofstream f("C:/Users/evanwaxman/Documents/workspace/rci_unit/sim/txt_files/UUT_testdata.txt");

		if (f.is_open()) {

			f << std::bitset<sizeof f * 8>(*(long unsigned int*)(&ray.dir.x));

			// create unions to convert data from float to binary
			///////////////////////ray unions
			/*union {
				float din;
				int dout;
			} dir_x;
			dir_x.din = ray.dir.x;

			union {
				float din;
				int dout;
			} dir_y;
			dir_y.din = ray.dir.y;

			union {
				float din;
				int dout;
			} dir_z;
			dir_z.din = ray.dir.z;

			// create bitset conversion
			std::bitset<sizeof(float) * CHAR_BIT> dirXbits(dir_x.dout);
			//std::bitset<sizeof(float) * CHAR_BIT> dirYbits(dir_y.dout);
			//std::bitset<sizeof(float) * CHAR_BIT> dirZbits(dir_z.dout);

			// output ray.dir to UUT_testdata.txt
			f << dirXbits << "_" << dirYbits << "_" << dirZbits << "_";*/
		}


        depth_scale = rsqrt(dot(ray.dir,ray.dir));
        LinearSpace3fa space = frame(depth_scale*ray.dir);
        space.vz *= depth_scale;
        ray_space = space.transposed();
		

		/*if (f.is_open()) {
			// create unions to convert data from float to binary
			///////////////////////depth_scale union
			union {
				float din;
				int dout;
			} depth;
			depth.din = depth_scale;

			///////////////////////ray_space unions
			union {
				float din;
				int dout;
			} ray_space_vx_x;
			ray_space_vx_x.din = ray_space.vx.x;

			union {
				float din;
				int dout;
			} ray_space_vx_y;
			ray_space_vx_y.din = ray_space.vx.y;

			union {
				float din;
				int dout;
			} ray_space_vx_z;
			ray_space_vx_z.din = ray_space.vx.z;

			union {
				float din;
				int dout;
			} ray_space_vy_x;
			ray_space_vy_x.din = ray_space.vy.x;

			union {
				float din;
				int dout;
			} ray_space_vy_y;
			ray_space_vy_y.din = ray_space.vy.y;

			union {
				float din;
				int dout;
			} ray_space_vy_z;
			ray_space_vy_z.din = ray_space.vy.z;

			union {
				float din;
				int dout;
			} ray_space_vz_x;
			ray_space_vz_x.din = ray_space.vz.x;

			union {
				float din;
				int dout;
			} ray_space_vz_y;
			ray_space_vz_y.din = ray_space.vz.y;

			union {
				float din;
				int dout;
			} ray_space_vz_z;
			ray_space_vz_z.din = ray_space.vz.z;

			// create bitset conversion
			std::bitset<sizeof(float) * CHAR_BIT> ray_space_vx_x_bits(ray_space_vx_x.dout);
			std::bitset<sizeof(float) * CHAR_BIT> ray_space_vx_y_bits(ray_space_vx_y.dout);
			std::bitset<sizeof(float) * CHAR_BIT> ray_space_vx_z_bits(ray_space_vx_z.dout);
			std::bitset<sizeof(float) * CHAR_BIT> ray_space_vy_x_bits(ray_space_vy_x.dout);
			std::bitset<sizeof(float) * CHAR_BIT> ray_space_vy_y_bits(ray_space_vy_y.dout);
			std::bitset<sizeof(float) * CHAR_BIT> ray_space_vy_z_bits(ray_space_vy_z.dout);
			std::bitset<sizeof(float) * CHAR_BIT> ray_space_vz_x_bits(ray_space_vx_x.dout);
			std::bitset<sizeof(float) * CHAR_BIT> ray_space_vz_y_bits(ray_space_vy_y.dout);
			std::bitset<sizeof(float) * CHAR_BIT> ray_space_vz_z_bits(ray_space_vz_z.dout);


			f << depth_scale << "_" << ray_space_vx_x_bits << "_" << ray_space_vx_y_bits << "_" << ray_space_vx_z_bits << "_"
				<< ray_space_vy_x_bits << "_" << ray_space_vy_y_bits << "_" << ray_space_vy_z_bits << "_"
				<< ray_space_vz_x_bits << "_" << ray_space_vz_y_bits << "_" << ray_space_vz_z_bits << "_" << std::endl;
		}*/

		f.close();
      }
    };
    
    template<int K>
      struct CurvePrecalculationsK
    {
      vfloat<K> depth_scale;
      LinearSpace3fa ray_space[K];

      __forceinline CurvePrecalculationsK(const vbool<K>& valid, const RayK<K>& ray)
      {
        size_t mask = movemask(valid);
        depth_scale = rsqrt(dot(ray.dir,ray.dir));
        while (mask) {
          size_t k = bscf(mask);
          Vec3fa ray_dir_k = Vec3fa(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]);
          LinearSpace3fa ray_space_k = frame(depth_scale[k]*ray_dir_k);
          ray_space_k.vz *= depth_scale[k];
          ray_space[k] = ray_space_k.transposed();
        }
      }
    };
  }
}
