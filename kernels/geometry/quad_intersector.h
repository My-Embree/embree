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

namespace embree
{
  namespace isa
  {
    /*! Intersects a ray with a quad with backface culling
     *  enabled. The quad v0,v1,v2,v3 is split into two triangles
     *  v0,v1,v3 and v2,v3,v1. The edge v1,v2 decides which of the two
     *  triangles gets intersected. */
    template<int N>
	__forceinline vbool<N> intersect_quad_backface_culling(const vbool<N>& valid0,
		const Vec3fa& ray_org,
		const Vec3fa& ray_dir,
		const float ray_tnear,
		const float ray_tfar,
		const Vec3vf<N>& quad_v0,
		const Vec3vf<N>& quad_v1,
		const Vec3vf<N>& quad_v2,
		const Vec3vf<N>& quad_v3,
		vfloat<N>& u_o,
		vfloat<N>& v_o,
		vfloat<N>& t_o)
	{
		static int inputSampleCount = 0;
		static int outputSampleCount = 0;


		// FOR IQBC TEST
		/*static int numInCount = 0;
		static int infInCount = 0;
		static int numOutCount = 0;
		static int infOutCount = 0;

		if (inputSampleCount < 100) {
			if (ray_tfar != std::numeric_limits<float>::infinity()) {
				if (numInCount < 50) {
					std::ofstream svInputFile;
					svInputFile.open("C:/Users/evanwaxman/Documents/workspace/rci_unit/sim/txt_files/iqbc_input_file.txt", std::ios::app);

					if (svInputFile.is_open()) {
						svInputFile << std::setprecision(16) << valid0.i[0] << " ";
						svInputFile << std::setprecision(16) << ray_tnear << " ";
						svInputFile << std::setprecision(16) << ray_tfar << " ";
						svInputFile << std::setprecision(16) << quad_v0.x[0] << " " << quad_v0.y[0] << " " << quad_v0.z[0] << " "
							<< quad_v1.x[0] << " " << quad_v1.y[0] << " " << quad_v1.z[0] << " "
							<< quad_v2.x[0] << " " << quad_v2.y[0] << " " << quad_v2.z[0] << " "
							<< quad_v3.x[0] << " " << quad_v3.y[0] << " " << quad_v3.z[0] << std::endl;
					}

					svInputFile.close();

					++inputSampleCount;
					++numInCount;
				}
				else {
					std::cout << "DONE SAMPLING" << std::endl;
				}
			} else {
				if (infInCount < 50) {
					std::ofstream svInputFile;
					svInputFile.open("C:/Users/evanwaxman/Documents/workspace/rci_unit/sim/txt_files/iqbc_input_file.txt", std::ios::app);

					if (svInputFile.is_open()) {
						svInputFile << std::setprecision(16) << valid0.i[0] << " ";
						svInputFile << std::setprecision(16) << ray_tnear << " ";
						svInputFile << std::setprecision(16) << ray_tfar << " ";
						svInputFile << std::setprecision(16) << quad_v0.x[0] << " " << quad_v0.y[0] << " " << quad_v0.z[0] << " "
							<< quad_v1.x[0] << " " << quad_v1.y[0] << " " << quad_v1.z[0] << " "
							<< quad_v2.x[0] << " " << quad_v2.y[0] << " " << quad_v2.z[0] << " "
							<< quad_v3.x[0] << " " << quad_v3.y[0] << " " << quad_v3.z[0] << std::endl;
					}

					svInputFile.close();

					++inputSampleCount;
					++infInCount;
				}
				else {
					std::cout << "DONE SAMPLING" << std::endl;
				}
			}
		}*/

      /* calculate vertices relative to ray origin */
      vbool<N> valid = valid0;
      const Vec3vf<N> O = Vec3vf<N>(ray_org);
      const Vec3vf<N> D = Vec3vf<N>(ray_dir);
      const Vec3vf<N> va = quad_v0-O;
      const Vec3vf<N> vb = quad_v1-O;
      const Vec3vf<N> vc = quad_v2-O;
      const Vec3vf<N> vd = quad_v3-O;

      const Vec3vf<N> edb = vb-vd;
      const vfloat<N> WW = dot(cross(vd,edb),D);
      const Vec3vf<N> v0 = select(WW <= 0.0f,va,vc);
      const Vec3vf<N> v1 = select(WW <= 0.0f,vb,vd);
      const Vec3vf<N> v2 = select(WW <= 0.0f,vd,vb);

      /* calculate edges */
      const Vec3vf<N> e0 = v2-v0;
      const Vec3vf<N> e1 = v0-v1;

      /* perform edge tests */
      const vfloat<N> U = dot(cross(v0,e0),D);
      const vfloat<N> V = dot(cross(v1,e1),D);
      valid &= max(U,V) <= 0.0f;
      if (unlikely(none(valid))) return false;

      /* calculate geometry normal and denominator */
      const Vec3vf<N> Ng = cross(e1,e0);
      const vfloat<N> den = dot(Ng,D);
      const vfloat<N> absDen = abs(den);
      const vfloat<N> sgnDen = signmsk(den);

      /* perform depth test */
      const vfloat<N> T = dot(v0,Ng);
      valid &= absDen*vfloat<N>(ray_tnear) < (T^sgnDen);
      valid &= (T^sgnDen) <= absDen*vfloat<N>(ray_tfar);
      if (unlikely(none(valid))) return false;

      /* avoid division by 0 */
      valid &= den != vfloat<N>(zero);
      if (unlikely(none(valid))) return false;

      /* update hit information */
      const vfloat<N> rcpDen = rcp(den);
      t_o = T * rcpDen;
      u_o = U * rcpDen;
      v_o = V * rcpDen;
      u_o = select(WW <= 0.0f,u_o,1.0f-u_o);
      v_o = select(WW <= 0.0f,v_o,1.0f-v_o);



	  // FOR IQBC TEST
	  /*if (outputSampleCount < 100) {
		  if (ray_tfar != std::numeric_limits<float>::infinity()) {
			  if (numOutCount < 50) {
				  std::ofstream svOutputFile;
				  svOutputFile.open("C:/Users/evanwaxman/Documents/workspace/rci_unit/sim/txt_files/iqbc_output_file.txt", std::ios::app);

				  if (svOutputFile.is_open()) {
					  svOutputFile << std::setprecision(16) << valid.i[0] << " ";
					  svOutputFile << std::setprecision(16) << u_o.f[0] << " ";
					  svOutputFile << std::setprecision(16) << v_o.f[0] << " ";
					  svOutputFile << std::setprecision(16) << t_o.f[0] << std::endl;
				  }

				  svOutputFile.close();

				  ++outputSampleCount;
				  ++numOutCount;
			  }
			  else {
				  std::cout << "DONE SAMPLING" << std::endl;
			  }
		  }
		  else {
			  if (infOutCount < 50) {
				  std::ofstream svOutputFile;
				  svOutputFile.open("C:/Users/evanwaxman/Documents/workspace/rci_unit/sim/txt_files/iqbc_output_file.txt", std::ios::app);

				  if (svOutputFile.is_open()) {
					  svOutputFile << std::setprecision(16) << valid.i[0] << " ";
					  svOutputFile << std::setprecision(16) << u_o.f[0] << " ";
					  svOutputFile << std::setprecision(16) << v_o.f[0] << " ";
					  svOutputFile << std::setprecision(16) << t_o.f[0] << std::endl;
				  }

				  svOutputFile.close();

				  ++outputSampleCount;
				  ++infOutCount;
			  }
			  else {
				  std::cout << "DONE SAMPLING" << std::endl;
			  }
		  }
	  }*/


      return valid;
    }
  }
}
