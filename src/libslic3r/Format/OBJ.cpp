#include "../libslic3r.h"
#include "../Model.hpp"
#include "../TriangleMesh.hpp"


#include "OBJ.hpp"
#include "objparser.hpp"

#include <png.h>

#include <string>
#include <boost/log/trivial.hpp>


#ifdef _WIN32
#define DIR_SEPARATOR '\\'
#else
#define DIR_SEPARATOR '/'
#endif

namespace Slic3r {
// add back for compatibility 
bool load_obj(const char *path, TriangleMesh *meshptr)
{
    if(meshptr == nullptr) return false;
    
    // Parse the OBJ file.
    ObjParser::ObjData data;
    if (! ObjParser::objparse(path, data)) {
        BOOST_LOG_TRIVIAL(error) << "load_obj: failed to parse " << path;
        return false;
    }
    
    // Count the faces and verify, that all faces are triangular.
    size_t num_faces = 0;
    size_t num_quads = 0;
    for (size_t i = 0; i < data.vertices.size(); ) {
        size_t j = i;
        for (; j < data.vertices.size() && data.vertices[j].coordIdx != -1; ++ j) ;
        if (i == j)
            continue;
        size_t face_vertices = j - i;
        if (face_vertices != 3 && face_vertices != 4) {
            // Non-triangular and non-quad faces are not supported as of now.
            return false;
        }
        if (face_vertices == 4)
            ++ num_quads;
        ++ num_faces;
        i = j + 1;
    }
    
    // Convert ObjData into STL.
    TriangleMesh &mesh = *meshptr;
    stl_file &stl = mesh.stl;
    stl.stats.type = inmemory;
    stl.stats.number_of_facets = uint32_t(num_faces + num_quads);
    stl.stats.original_num_facets = int(num_faces + num_quads);
    // stl_allocate clears all the allocated data to zero, all normals are set to zeros as well.
    stl_allocate(&stl);
    size_t i_face = 0;
    for (size_t i = 0; i < data.vertices.size(); ++ i) {
        if (data.vertices[i].coordIdx == -1)
            continue;
        stl_facet &facet = stl.facet_start[i_face ++];
        size_t     num_normals = 0;
        stl_normal normal(stl_normal::Zero());
        for (unsigned int v = 0; v < 3; ++ v) {
            const ObjParser::ObjVertex &vertex = data.vertices[i++];
            memcpy(facet.vertex[v].data(), &data.coordinates[vertex.coordIdx*4], 3 * sizeof(float));
            if (vertex.normalIdx != -1) {
                normal(0) += data.normals[vertex.normalIdx*3];
                normal(1) += data.normals[vertex.normalIdx*3+1];
                normal(2) += data.normals[vertex.normalIdx*3+2];
                ++ num_normals;
            }
        }
        // Result of obj_parseline() call is not checked, thus not all vertices are necessarily finalized with coord_Idx == -1.
        if (i < data.vertices.size() && data.vertices[i].coordIdx != -1) {
            // This is a quad. Produce the other triangle.
            stl_facet &facet2 = stl.facet_start[i_face++];
            facet2.vertex[0] = facet.vertex[0];
            facet2.vertex[1] = facet.vertex[2];
            const ObjParser::ObjVertex &vertex = data.vertices[i++];
            memcpy(facet2.vertex[2].data(), &data.coordinates[vertex.coordIdx * 4], 3 * sizeof(float));
            if (vertex.normalIdx != -1) {
                normal(0) += data.normals[vertex.normalIdx*3];
                normal(1) += data.normals[vertex.normalIdx*3+1];
                normal(2) += data.normals[vertex.normalIdx*3+2];
                ++ num_normals;
            }
            if (num_normals == 4) {
                // Normalize an average normal of a quad.
                float len = facet.normal.norm();
                if (len > EPSILON) {
                    normal /= len;
                    facet.normal = normal;
                    facet2.normal = normal;
                }
            }
        } else if (num_normals == 3) {
            // Normalize an average normal of a triangle.
            float len = facet.normal.norm();
            if (len > EPSILON)
                facet.normal = normal / len;
        }
    }
    stl_get_size(&stl);
    mesh.repair();
    if (mesh.facets_count() == 0) {
        BOOST_LOG_TRIVIAL(error) << "load_obj: This OBJ file couldn't be read because it's empty. " << path;
        return false;
    }
    
    return true;
}

bool mtl_readPNG(char* path, png_structp png_ptr, png_infop info_ptr){
    int number=4;
    char* header[200];
    
    FILE *f= fopen(path,"rb");
    if (!f)
        return false;
    if (fread(header, 1, number, f) != number)
    {
       return false;
    }

    //int is_png = !png_sig_cmp(*header, 0, number);
    //if (!is_png)
    //{
      //  error();
    //}
    
    char buf[128];
    png_ptr = png_create_read_struct
        (PNG_LIBPNG_VER_STRING, (png_voidp)buf,
        (png_error_ptr)NULL, (png_error_ptr)NULL);
    if(!png_ptr){
        return false;
    }
    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr){
       png_destroy_read_struct(&png_ptr,(png_infopp)NULL, (png_infopp)NULL);
       return false;
    }
    /*if (setjmp(png_jmpbuf(png_ptr)))
    {
       png_destroy_read_struct(&png_ptr, &info_ptr,
           &end_info);
       fclose(f);
       error();
    }*/
    
    png_init_io(png_ptr, f);
    png_set_sig_bytes(png_ptr, number);
    png_set_compression_buffer_size(png_ptr, 4096*2);
    png_read_png(png_ptr, info_ptr, 0, NULL);
    
    png_infop end_info = png_create_info_struct(png_ptr);
    //png_read_end(png_ptr,end_info);
    fclose(f);
    return true;
}
struct color{
    float c;//rotation around center:0-1
    float s;//distance from center:0-1
    float v;//distance from 0:0-1

    float r;
    float g;
    float b;
    void RGBtoCSV(){
        //a triangle
        if(r>b&&r>g){
            s=r-(g*b);
            c=r-b/3.0+g/3.0;
            v=r;
        }
        else if(g>b&&g>r){
            s=g-(g*b);
            c=1.0/3.0+g-r/3.0+b/3.0;
            v=g;
        }
        else if(b>r&&b>g){
            s=b-(g*r);
            c=(2.0/3.0)*b+r/3.0-g/3.0;
            v=b;
        }
        //clamp c,s,v:
        if(s>1)
            s=1;
        if(s<0)
            s=0;
        if(v>1)
            v=1;
        if(v<0)
            v=0;
        while(c>1)
            c-=1.0;
        while(c<0)
            c+=1.0;     
    }
    void R(float R){
        r=R;
        if(r>1)
            r=1;
        if(r<0)
            r=0;
    }
    void G(float G){
        g=G;
        if(g>1)
            g=1;
        if(g<0)
            g=0;
    }
    void B(float B){
        b=B;
        if(b>1)
            b=1;
        if(b<0)
            b=0;
    }

    void C(float C){
        c=C;
        while(c>1)
            c-=1.0;
        while(c<0)
            c+=1.0;
    }
    void S(float S){
        s=S;
        if(s>1)
            s=1;
        if(s<0)
            s=0;
    }
    void V(float V){
        v=V;
        if(v>1)
            v=1;
        if(v<0)
            v=0;
    }
};
int p(int m,int p){
    int out=1;
    while(p>0){
        out*=m;
        p--;
    }
    return out;
}
bool load_obj(const char *path, TriangleMesh *meshptr, Model *model,const char *object_name_in)
{
    if(meshptr == nullptr) return false;
    
    // Parse the OBJ file.
    ObjParser::ObjData data;
    if (! ObjParser::objparse(path, data)) {
        BOOST_LOG_TRIVIAL(error) << "load_obj: failed to parse " << path;
        return false;
    }
    
    // Count the faces and verify, that all faces are triangular.
    size_t num_faces = 0;
    size_t num_quads = 0;
    for (size_t i = 0; i < data.vertices.size(); ) {
        size_t j = i;
        for (; j < data.vertices.size() && data.vertices[j].coordIdx != -1; ++ j) ;
        if (i == j)
            continue;
        size_t face_vertices = j - i;
        if (face_vertices != 3 && face_vertices != 4) {
            // Non-triangular and non-quad faces are not supported as of now.
            return false;
        }
        if (face_vertices == 4)
            ++ num_quads;
        ++ num_faces;
        i = j + 1;
    }
    
    // Convert ObjData into STL.
    TriangleMesh &mesh = *meshptr;
    stl_file &stl = mesh.stl;
    uv_data uvData;
    mtl_file mtl;
    //indexed_triangle_set &its = mesh.its;
    stl.stats.type = inmemory;
    stl.stats.number_of_facets = uint32_t(num_faces + num_quads);
    stl.stats.original_num_facets = int(num_faces + num_quads);
    // stl_allocate clears all the allocated data to zero, all normals are set to zeros as well.
    stl_allocate(&stl);
    size_t i_face = 0;
    for (size_t i = 0; i < data.vertices.size(); ++ i) {
        if (data.vertices[i].coordIdx == -1)
            continue;
        stl_facet &facet = stl.facet_start[i_face ++];
        size_t     num_normals = 0;
        stl_normal normal(stl_normal::Zero());
        size_t     num_uvs = 0;
        stl_uv uv[3];
        
        for (unsigned int v = 0; v < 3; ++ v) {
            const ObjParser::ObjVertex &vertex = data.vertices[i++];
            memcpy(facet.vertex[v].data(), &data.coordinates[vertex.coordIdx*4], 3 * sizeof(float));
            if (vertex.normalIdx != -1) {
                normal(0) += data.normals[vertex.normalIdx*3];
                normal(1) += data.normals[vertex.normalIdx*3+1];
                normal(2) += data.normals[vertex.normalIdx*3+2];
                ++ num_normals;
            }
            if (vertex.textureCoordIdx != -1) {
                uv[v][0] += data.textureCoordinates[vertex.textureCoordIdx*3];
                uv[v][1] += data.textureCoordinates[vertex.textureCoordIdx*3+1];
                uv[v][2] += data.textureCoordinates[vertex.textureCoordIdx*3+2];
                ++ num_uvs;
            }
            
        }
        
        // Result of obj_parseline() call is not checked, thus not all vertices are necessarily finalized with coord_Idx == -1.
        if (i < data.vertices.size() && data.vertices[i].coordIdx != -1) {
            // This is a quad. Produce the other triangle.
            stl_facet &facet2 = stl.facet_start[i_face++];
            facet2.vertex[0] = facet.vertex[0];
            facet2.vertex[1] = facet.vertex[2];
            
            const ObjParser::ObjVertex &vertex = data.vertices[i++];
            memcpy(facet2.vertex[2].data(), &data.coordinates[vertex.coordIdx * 4], 3 * sizeof(float));
            if (vertex.normalIdx != -1) {
                normal(0) += data.normals[vertex.normalIdx*3];
                normal(1) += data.normals[vertex.normalIdx*3+1];
                normal(2) += data.normals[vertex.normalIdx*3+2];
                ++ num_normals;
            }
            if (num_normals == 4) {
                // Normalize an average normal of a quad.
                float len = facet.normal.norm();
                if (len > EPSILON) {
                    normal /= len;
                    facet.normal = normal;
                    facet2.normal = normal;
                    
                }
            }
            
        } else if (num_normals == 3) {
            
            // Normalize an average normal of a triangle.
            float len = facet.normal.norm();
            if (len > EPSILON)
                facet.normal = normal / len;
        }
        //should work
        uvData.uvs.emplace_back(uv[0]);
        uvData.uvs.emplace_back(uv[1]);
        uvData.uvs.emplace_back(uv[2]);
    }
    std::string  object_name;
    if (object_name_in == nullptr) {
        const char *last_slash = strrchr(path, DIR_SEPARATOR);
        object_name.assign((last_slash == nullptr) ? path : last_slash + 1);
    } else
       object_name.assign(object_name_in);
    
    model->add_object(object_name.c_str(), path, std::move(mesh));
    //load mtl data
    ObjParser::parseMTL(path, mtl, data);
    int x1;
    //load images, map uvs, and add triangles of different colors
    for(int i=0;i<stl.facet_start.size();i++){
        png_structp png_ptr;
        png_infop info_ptr;
        //select image;
        for(int i2=0;i2<64;++i2){
            if(i>mtl.idx[i2]){
                x1=i2;
            }
        }
        int i2=x1;
        mtl_readPNG(mtl.png[mtl.mtl_idx[i2]],png_ptr,info_ptr);
        png_bytepp rows = png_get_rows(png_ptr,info_ptr);
        png_uint_32 width = png_get_image_width(png_ptr,info_ptr);
        png_uint_32 height = png_get_image_height(png_ptr,info_ptr);
        png_byte channels = png_get_channels(png_ptr,info_ptr);
        png_set_palette_to_rgb(png_ptr);
        
        //map uvs
        int uv1[2] = {uvData.uvs[i*3][0]*width,uvData.uvs[i*3][1]*height};
        int uv2[2] = {uvData.uvs[i*3+1][0]*width,uvData.uvs[i*3+1][1]*height};
        int uv3[2] = {uvData.uvs[i*3+2][0]*width,uvData.uvs[i*3+2][1]*height};
        float slp1=(uv1[1]-uv2[1])/(uv1[0]-uv2[0]);
        float slp2=(uv2[1]-uv3[1])/(uv2[0]-uv3[0]);
        float slp3=(uv1[1]-uv3[1])/(uv1[0]-uv3[0]);
        int iclr1=sqrt(   (uv1[1]-uv2[1])*(uv1[1]-uv2[1]) +   (uv1[0]-uv2[0])*(uv1[0]-uv2[0]) );
        int iclr2=sqrt(   (uv3[1]-uv2[1])*(uv3[1]-uv2[1]) +   (uv3[0]-uv2[0])*(uv3[0]-uv2[0]) );
        int iclr3=sqrt(   (uv1[1]-uv3[1])*(uv1[1]-uv3[1]) +   (uv1[0]-uv3[0])*(uv1[0]-uv3[0]) );
        //make all sides of the triangle the same length(equilateral, for easy conversion to what is presumably subdivided(simple) triangles)
        int trsil;
        if(iclr1>iclr2&&iclr1>iclr3){
            trsil=iclr1;
        }
        else if(iclr2>iclr3&&iclr2>iclr1){
            trsil=iclr2;
        }
        else{
            trsil=iclr3;
        }
        //rebase trsil(longest side of triangle) to closest power of 2
        int mag=0;
        while(trsil>0){
            mag+=1;
            if(trsil==p(mag,2)){
                trsil=-1;
            }
            trsil-=trsil%(p(mag,2));
            
        }
        mag--;
        trsil=p(mag,2);
    
        color clr1[trsil];
        color clr2[trsil];
        color clr3[trsil];
        //set color lines
        {
            int nX=uv1[0];
            int nY=uv1[1];
            int v=0;
            while(v<iclr1){
                for(int v1=0;v1<abs(uv1[1]-uv2[1]);++v1)//y
                {
                    if(slp1>0){
                        nY++;
                    }else{
                        nY--;
                    }
                    clr1[v].R(float(rows[nY*3][nX]/256.0));
                    clr1[v].G(float(rows[nY*3+1][nX]/256.0));
                    clr1[v].B(float(rows[nY*3+2][nX]/256.0));
                    clr1[v].RGBtoCSV();
                    v++;

                }
                for(int v1=0;v1<abs(uv1[0]-uv2[0]);++v1)//x
                {
                    nX++;
                    clr1[v].R(float(rows[nY*3][nX]/256.0));
                    clr1[v].G(float(rows[nY*3+1][nX]/256.0));
                    clr1[v].B(float(rows[nY*3+2][nX]/256.0));
                    clr1[v].RGBtoCSV();
                    v++;
                }
            }
        }

        {
            int nX=uv1[0];
            int nY=uv1[1];
            int v=0;
            while(v<iclr1){
                for(int v1=0;v1<abs(uv3[1]-uv2[1]);++v1)//y
                {
                    if(slp2>0){
                        nY++;
                    }else{
                        nY--;
                    }
                    clr2[v].R(float(rows[nY*3][nX]/256.0));
                    clr2[v].G(float(rows[nY*3+1][nX]/256.0));
                    clr2[v].B(float(rows[nY*3+2][nX]/256.0));
                    clr2[v].RGBtoCSV();
                    v++;

                }
                for(int v1=0;v1<abs(uv3[0]-uv2[0]);++v1)//x
                {
                    nX++;
                    clr2[v].R(float(rows[nY*3][nX]/256.0));
                    clr2[v].G(float(rows[nY*3+1][nX]/256.0));
                    clr2[v].B(float(rows[nY*3+2][nX]/256.0));
                    clr2[v].RGBtoCSV();
                    v++;
                }
            }
        }

        {
            int nX=uv1[0];
            int nY=uv1[1];
            int v=0;
            while(v<iclr3){
                for(int v1=0;v1<abs(uv1[1]-uv3[1]);++v1)//y
                {
                    if(slp3>0){
                        nY++;
                    }else{
                        nY--;
                    }
                    clr3[v].R(float(rows[nY*3][nX]/256.0));
                    clr3[v].G(float(rows[nY*3+1][nX]/256.0));
                    clr3[v].B(float(rows[nY*3+2][nX]/256.0));
                    clr3[v].RGBtoCSV();
                    v++;

                }
                for(int v1=0;v1<abs(uv1[0]-uv3[0]);++v1)//x
                {
                    nX++;
                    clr3[v].R(float(rows[nY*3][nX]/256.0));
                    clr3[v].G(float(rows[nY*3+1][nX]/256.0));
                    clr3[v].B(float(rows[nY*3+2][nX]/256.0));
                    clr3[v].RGBtoCSV();
                    v++;
                }
            }
        }
        //expand color lines
        {
            color Tclr1[trsil];
            color Tclr2[trsil];
            color Tclr3[trsil];
            for(int b1=0;b1<iclr1;++b1){
                Tclr1[(int)(b1*(trsil/iclr1))]=clr1[b1];
            }
            for(int b1=0;b1<trsil;++b1){
                clr1[b1]=Tclr1[b1];
            }
            for(int b1=0;b1<iclr2;++b1){
                Tclr2[(int)(b1*(trsil/iclr2))]=clr2[b1];
            }
            for(int b1=0;b1<trsil;++b1){
                clr2[b1]=Tclr2[b1];
            }
            for(int b1=0;b1<iclr3;++b1){
                Tclr3[(int)(b1*(trsil/iclr3))]=clr3[b1];
            }
            for(int b1=0;b1<trsil;++b1){
                clr3[b1]=Tclr3[b1];
            }
            
        }
        //..
        //split triangle data
        //0011=split 
        //fractal
        //1-4-16-64

        
        std::pair<std::vector<std::pair<int, int>>, std::vector<bool>> data;
        assert(data.first.empty() || data.first.back().first < i);
        data.first.emplace_back(i,int(data.second.size()));
        for(int id=0; id<mag; id++){
            data.second.push_back(0);
            data.second.push_back(0);
            data.second.push_back(1);
            data.second.push_back(1);
        }
        //test
        for(int id=0; id<p(mag,4);id++){
            data.second.push_back(0);
            data.second.push_back(0);
            data.second.push_back(0);
            data.second.push_back(1);
        }
        std::string STR="";
        for(int n=0;n<data.second.size();n+=4){
            char ch=data.second[n]*0x1+data.second[n+1]*0x2+data.second[n+2]*0x4+data.second[n+3]*0x8;
            STR[n]=ch;
        }
        //what a terrible workaround. 
        model->objects[0]->volumes[0]->mmu_segmentation_facets.set_triangle_from_string(i,STR);
    }


      
    
     
    stl_get_size(&stl);
    mesh.repair();
    if (mesh.facets_count() == 0) {
        BOOST_LOG_TRIVIAL(error) << "load_obj: This OBJ file couldn't be read because it's empty. " << path;
        return false;
    }
    
    return true;
}

bool load_obj(const char *path, Model *model, const char *object_name_in)
{
    TriangleMesh mesh;
    bool ret = load_obj(path, &mesh, model,object_name_in);
    
    if (ret) {
        
        
    }
    
    return ret;
}

bool store_obj(const char *path, TriangleMesh *mesh)
{
    //FIXME returning false even if write failed.
    mesh->WriteOBJFile(path);
    return true;
}

bool store_obj(const char *path, ModelObject *model_object)
{
    TriangleMesh mesh = model_object->mesh();
    return store_obj(path, &mesh);
}

bool store_obj(const char *path, Model *model)
{
    TriangleMesh mesh = model->mesh();
    return store_obj(path, &mesh);
}

}; // namespace Slic3r