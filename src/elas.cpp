#include "elas.h"

#include <algorithm>
#include <math.h>
#include "triangle.h"
#include "matrix.h"
#include <arm_neon.h>
#include <sys/time.h>


const double CLK_TCK = 1000.0;

//#include <CL/cl.h>

using namespace std;

struct support_pt {
  int32_t u;
  int32_t v;
  int32_t d;
  support_pt(int32_t u,int32_t v,int32_t d):u(u),v(v),d(d){}
};

struct triangle {
  int32_t c1,c2,c3;
  float   t1a,t1b,t1c;
  float   t2a,t2b,t2c;
  triangle(int32_t c1,int32_t c2,int32_t c3):c1(c1),c2(c2),c3(c3){}
};


extern void* HostMal(void **p, long size);
extern void cudaFreeHost_cpuaa(void *p);
extern void cuda_computeD(int32_t* disparity_grid_1, int32_t* disparity_grid_2,  vector<Elas::support_pt> &p_support, \
              vector<Elas::triangle> &tri_1, vector<Elas::triangle> &tri_2, \
              float* D1, float* D2,uint8_t* I1, uint8_t* I2, int8_t* P_g,\
                          int8_t *tp1_g, int8_t* tp2_g, int8_t* tp1_c, int8_t* tp2_c, float* cloud_g);

//extern vector<Elas::support_pt> computeSupportMatches_g(uint8_t* I_desc1, uint8_t* I_desc2) ;
extern vector<Elas::support_pt> computeSupportMatches_g(uint8_t* I1_desc, uint8_t* I2_desc,\
                                  int8_t* D_sup_c, int8_t* D_sup_g);

extern int ConvertD2Z(float* D1_g,  float* PointXYZ);

#define GRID_WIDTH      16
#define GRID_HEIGH      12
#define WIDTH           320
#define HEIGH           240
#define D_CAN_WIDTH     60
#define D_CAN_HEIGH     48

Elas::Elas(parameters param, int32_t width, int32_t height, int32_t D_can_width, int32_t D_can_height)
:param(param),desc_1(width, height, width),\
  desc_2(width,height,width)
{
//

    D_sup_g         = (int8_t*)HostMal((void**)&D_sup_c, D_can_width*D_can_height * sizeof(int8_t) );
    memset(D_sup_c, -1, D_can_width*D_can_height * sizeof(int8_t));
//    D1_data_g       = (float*)HostMal((void**)&D1_data_c, width * height * sizeof(float) * 3);
//    D2_data_g       = (float*)HostMal((void**)&D2_data_c, width * height * sizeof(float) * 3);

    D1_data_g       = (float*)HostMal((void**)&D1_data_c, width * height * sizeof(float));
    D2_data_g       = (float*)HostMal((void**)&D2_data_c, width * height * sizeof(float));

    disp_grid_1_g   = (int32_t*)HostMal((void**)&disp_grid_1_c,  65 * GRID_WIDTH * GRID_HEIGH * sizeof(int32_t));
    disp_grid_2_g   = (int32_t*)HostMal((void**)&disp_grid_2_c,  65 * GRID_WIDTH * GRID_HEIGH * sizeof(int32_t));
    tp1_g           = (int8_t*)HostMal((void**)&tp1_c, width * height * sizeof(int8_t) );
    tp2_g           = (int8_t*)HostMal((void**)&tp2_c, width * height * sizeof(int8_t) );

    P_g             = (int8_t*)HostMal((void**)&P_c, 64 * sizeof(int8_t));

    int8_t temp[] = {-14,-9,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    for(int i = 0 ; i < 64; i++){
        P_c[i] = temp[i];
    }
   cloud_g         = (point*)HostMal((void**)&cloud_c, width * height * sizeof(float) * 3 );
}





void Elas::process (uint8_t* I1_,uint8_t* I2_)
{

      struct timeval start, end;
      clock_t t1,t2;
      width  = WIDTH;// dims[0];
      height = HEIGH; // dims[1];
      bpl    = width + 15-(width-1)%16;
      double timeuse;
      I1 = I1_;
      I2 = I2_;

      gettimeofday(&start, NULL);
      desc_1.compute(I1_,width,height,bpl,param.subsampling);
      gettimeofday(&end, NULL);
      timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
      cout << "two desc1: " << timeuse/1000 << "ms" <<endl;

            gettimeofday(&start, NULL);
      desc_2.compute(I2_,width,height,bpl,param.subsampling);
      gettimeofday(&end, NULL);
      timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
      cout << "two desc2: " << timeuse/1000 << "ms" <<endl;

      gettimeofday(&start, NULL);
      vector<support_pt> p_support = computeSupportMatches_g(desc_1.I_desc_g, desc_2.I_desc_g,  D_sup_c, D_sup_g);
      //computeSupportMatches_g(desc_1.I_desc_g, desc_2.I_desc_g,  D_sup_c, D_sup_g);
      
//      vector<support_pt> p_support;
//	for(int v_can = 0; v_can < 48; v_can++)
//		for( int u_can = 0; u_can < 60; u_can++)
//			if(*(D_sup_c + u_can + v_can * 60) >= 0)
//			p_support.push_back(Elas::support_pt((u_can+3) * 5, (v_can+1) * 5, *(D_sup_c + u_can + v_can * 60)));
	memset(D_sup_c, -1, D_CAN_WIDTH*D_CAN_HEIGH * sizeof(int8_t));
     //      vector<support_pt> p_support = computeSupportMatches(desc_1.I_desc, desc_2.I_desc);
      gettimeofday(&end, NULL);
      timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
      cout <<"computesuppportmatch: "<< timeuse/1000 << "ms. " ; cout << "support size: "<<p_support.size() <<endl;

      // if not enough support points for triangulation
      if (p_support.size()<3)
      {
        cout << "ERROR: Need at least 3 support points!" << endl;
        return;
      }

      gettimeofday(&start, NULL);
      vector<triangle> tri_1 = computeDelaunayTriangulation(p_support,0);
      vector<triangle> tri_2 = computeDelaunayTriangulation(p_support,1);
      gettimeofday(&end, NULL);
      timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
      cout <<"Delaunay : "<< timeuse/1000 << "ms" <<endl;

      gettimeofday(&start, NULL);
      computeDisparityPlanes(p_support,tri_1,0);
      computeDisparityPlanes(p_support,tri_2,1);
      gettimeofday(&end, NULL);
      timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
      cout <<"computediparityplanes: "<< timeuse/1000 << "ms" <<endl;


      // allocate memory for disparity grid
      int32_t grid_width   = 16; //int32_t)ceil((float)width/(float)param.grid_size);
      int32_t grid_height  = 12; //(int32_t)ceil((float)height/(float)param.grid_size);
      int32_t grid_dims[3] = {param.disp_max+2,grid_width,grid_height};

      gettimeofday(&start, NULL);
      createGrid(p_support, disp_grid_1_c, grid_dims,0);
      createGrid(p_support, disp_grid_2_c, grid_dims,1);
      gettimeofday(&end, NULL);
      timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
      cout <<"creategrid: "<< timeuse/1000 << "ms" <<endl;


      gettimeofday(&start, NULL);
      cuda_computeD(disp_grid_1_g, disp_grid_1_g, p_support, tri_1, tri_2, D1_data_g, D2_data_g,\
                    desc_1.I_desc_g, desc_2.I_desc_g, P_g, tp1_g, tp2_g, tp1_c, tp2_c , (float*) cloud_g);
      gettimeofday(&end, NULL);
      timeuse = 1000000* (end.tv_sec-start.tv_sec) + end.tv_usec-start.tv_usec;
      cout <<"cuda_computeD: "<< timeuse/1000 << "ms" <<endl;

}


vector<Elas::triangle> Elas::computeDelaunayTriangulation (vector<support_pt> p_support,int32_t right_image) {

  // input/output structure for triangulation
  struct triangulateio in, out;
  int32_t k;

  // inputs
  in.numberofpoints = p_support.size();
  in.pointlist = (float*)malloc(in.numberofpoints*2*sizeof(float));
  k=0;
  if (!right_image) {
    for (int32_t i=0; i<p_support.size(); i++) {
      in.pointlist[k++] = p_support[i].u;
      in.pointlist[k++] = p_support[i].v;
    }
  } else {
    for (int32_t i=0; i<p_support.size(); i++) {
      in.pointlist[k++] = p_support[i].u-p_support[i].d;
      in.pointlist[k++] = p_support[i].v;
    }
  }
  in.numberofpointattributes = 0;
  in.pointattributelist      = NULL;
  in.pointmarkerlist         = NULL;
  in.numberofsegments        = 0;
  in.numberofholes           = 0;
  in.numberofregions         = 0;
  in.regionlist              = NULL;
  
  // outputs
  out.pointlist              = NULL;
  out.pointattributelist     = NULL;
  out.pointmarkerlist        = NULL;
  out.trianglelist           = NULL;
  out.triangleattributelist  = NULL;
  out.neighborlist           = NULL;
  out.segmentlist            = NULL;
  out.segmentmarkerlist      = NULL;
  out.edgelist               = NULL;
  out.edgemarkerlist         = NULL;

  // do triangulation (z=zero-based, n=neighbors, Q=quiet, B=no boundary markers)
  char parameters[] = "zQB";
  triangulate(parameters, &in, &out, NULL);
  
  // put resulting triangles into vector tri
  vector<triangle> tri;
  k=0;
  for (int32_t i=0; i<out.numberoftriangles; i++) {
    tri.push_back(triangle(out.trianglelist[k],out.trianglelist[k+1],out.trianglelist[k+2]));
    k+=3;
  }
  
  // free memory used for triangulation
  free(in.pointlist);
  free(out.pointlist);
  free(out.trianglelist);
  
  // return triangles
  return tri;
}

void Elas::computeDisparityPlanes (vector<support_pt> p_support, vector<triangle> &tri, int32_t right_image) {

  // init matrices
  Matrix A(3,3);
  Matrix b(3,1);
//  printf("tri.size: %d\n", tri.size());
  // for all triangles do
  for (int32_t i=0; i<tri.size(); i++) {
    
    // get triangle corner indices
    int32_t c1 = tri[i].c1;
    int32_t c2 = tri[i].c2;
    int32_t c3 = tri[i].c3;
    
    // compute matrix A for linear system of left triangle
    A.val[0][0] = p_support[c1].u;
    A.val[1][0] = p_support[c2].u;
    A.val[2][0] = p_support[c3].u;
    A.val[0][1] = p_support[c1].v; A.val[0][2] = 1;
    A.val[1][1] = p_support[c2].v; A.val[1][2] = 1;
    A.val[2][1] = p_support[c3].v; A.val[2][2] = 1;
    
    // compute vector b for linear system (containing the disparities)
    b.val[0][0] = p_support[c1].d;
    b.val[1][0] = p_support[c2].d;
    b.val[2][0] = p_support[c3].d;
    
    // on success of gauss jordan elimination
    if (b.solve(A)) {
      
      // grab results from b
      tri[i].t1a = b.val[0][0];
      tri[i].t1b = b.val[1][0];
      tri[i].t1c = b.val[2][0];
      
    // otherwise: invalid
    } else {
      tri[i].t1a = 0;
      tri[i].t1b = 0;
      tri[i].t1c = 0;
    }
//	cout<<"left:"<<tri[i].t1a;
    // compute matrix A for linear system of right triangle
    A.val[0][0] = p_support[c1].u-p_support[c1].d;
    A.val[1][0] = p_support[c2].u-p_support[c2].d;
    A.val[2][0] = p_support[c3].u-p_support[c3].d;
    A.val[0][1] = p_support[c1].v; A.val[0][2] = 1;
    A.val[1][1] = p_support[c2].v; A.val[1][2] = 1;
    A.val[2][1] = p_support[c3].v; A.val[2][2] = 1;
    
    // compute vector b for linear system (containing the disparities)
    b.val[0][0] = p_support[c1].d;
    b.val[1][0] = p_support[c2].d;
    b.val[2][0] = p_support[c3].d;
    
    // on success of gauss jordan elimination
    if (b.solve(A)) {
      
      // grab results from b
      tri[i].t2a = b.val[0][0];
      tri[i].t2b = b.val[1][0];
      tri[i].t2c = b.val[2][0];
      
    // otherwise: invalid
    } else {
      tri[i].t2a = 0;
      tri[i].t2b = 0;
      tri[i].t2c = 0;
    }
//	cout<<"right:"<<tri[i].t2a<<endl;
  }  
}

void Elas::createGrid(vector<support_pt> p_support,int32_t* disparity_grid,int32_t* grid_dims,bool right_image) {
  
  // get grid dimensions
  int32_t grid_width  = grid_dims[1];
  int32_t grid_height = grid_dims[2];
  
  // allocate temporary memory
  int32_t* temp1 = (int32_t*)calloc((param.disp_max+1)*grid_height*grid_width,sizeof(int32_t));
  int32_t* temp2 = (int32_t*)calloc((param.disp_max+1)*grid_height*grid_width,sizeof(int32_t));
  
  // for all support points do
  for (int32_t i=0; i<p_support.size(); i++) {
    
    // compute disparity range to fill for this support point
    int32_t x_curr = p_support[i].u;
    int32_t y_curr = p_support[i].v;
    int32_t d_curr = p_support[i].d;
    int32_t d_min  = max(d_curr-1,0);
    int32_t d_max  = min(d_curr+1,param.disp_max);
    
    // fill disparity grid helper
    for (int32_t d=d_min; d<=d_max; d++) {
      int32_t x;
      if (!right_image)
        x = floor((float)(x_curr/param.grid_size));
      else
        x = floor((float)(x_curr-d_curr)/(float)param.grid_size);
      int32_t y = floor((float)y_curr/(float)param.grid_size);
      
      // point may potentially lay outside (corner points)
      if (x>=0 && x<grid_width &&y>=0 && y<grid_height) {
        int32_t addr = getAddressOffsetGrid(x,y,d,grid_width,param.disp_max+1);
        *(temp1+addr) = 1;
      }
    }
  }
  
  // diffusion pointers
  const int32_t* tl = temp1 + (0*grid_width+0)*(param.disp_max+1);
  const int32_t* tc = temp1 + (0*grid_width+1)*(param.disp_max+1);
  const int32_t* tr = temp1 + (0*grid_width+2)*(param.disp_max+1);
  const int32_t* cl = temp1 + (1*grid_width+0)*(param.disp_max+1);
  const int32_t* cc = temp1 + (1*grid_width+1)*(param.disp_max+1);
  const int32_t* cr = temp1 + (1*grid_width+2)*(param.disp_max+1);
  const int32_t* bl = temp1 + (2*grid_width+0)*(param.disp_max+1);
  const int32_t* bc = temp1 + (2*grid_width+1)*(param.disp_max+1);
  const int32_t* br = temp1 + (2*grid_width+2)*(param.disp_max+1);
  
  int32_t* result    = temp2 + (1*grid_width+1)*(param.disp_max+1); 
  int32_t* end_input = temp1 + grid_width*grid_height*(param.disp_max+1);
  
  // diffuse temporary grid
  for( ; br != end_input; tl++, tc++, tr++, cl++, cc++, cr++, bl++, bc++, br++, result++ )
    *result = *tl | *tc | *tr | *cl | *cc | *cr | *bl | *bc | *br;
  
  // for all grid positions create disparity grid
  for (int32_t x=0; x<grid_width; x++) {
    for (int32_t y=0; y<grid_height; y++) {
        
      // start with second value (first is reserved for count)
      int32_t curr_ind = 1;
      
      // for all disparities do
      for (int32_t d=0; d<=param.disp_max; d++) {

        // if yes => add this disparity to current cell
        if (*(temp2+getAddressOffsetGrid(x,y,d,grid_width,param.disp_max+1))>0) {
          *(disparity_grid+getAddressOffsetGrid(x,y,curr_ind,grid_width,param.disp_max+2))=d;
          curr_ind++;
        }
      }
      
      // finally set number of indices
      *(disparity_grid+getAddressOffsetGrid(x,y,0,grid_width,param.disp_max+2))=curr_ind-1;
    }
  }
  
  // release temporary memory
  free(temp1);
  free(temp2);
}
