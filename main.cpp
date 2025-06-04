#include <iostream>
#include <string>
#include <map>

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <vtkXMLUnstructuredGridReader.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkSmartPointer.h>
#include <vtkDataSetMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkUnstructuredGrid.h>
#include <vtkPolyData.h>
#include <vtkHexahedron.h>
#include <vtkSphereSource.h>
#include <vtkAppendPolyData.h>
#include <vtkFloatArray.h>
#include <vtkCellData.h>
#include <vtkLine.h>
#include <vtkCellLocator.h>
#include <vtkPointLocator.h>
#include <vtkCellCenters.h>

using namespace std;

struct point3d {
    uint32_t id;
    double pos[3];
};

void read_alg_mesh (const char filename[], vector<struct point3d> &mesh_points_alg) {
    printf("[!] Reading ALG mesh ...\n");
    FILE *file = fopen(filename, "r");
    uint32_t k = 0;
    double aux[17];
    while (fscanf(file,"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",\
            &aux[0],&aux[1],&aux[2],&aux[3],&aux[4],&aux[5],&aux[6],&aux[7],&aux[8],&aux[9],&aux[10],&aux[11],\
            &aux[12],&aux[13],&aux[14],&aux[15],&aux[16]) != EOF) {
        struct point3d p;
        p.id = k;
        p.pos[0] = aux[0];
        p.pos[1] = aux[1];
        p.pos[2] = aux[2];
        k++;

        mesh_points_alg.push_back(p);
    }
    fclose(file);
}

void read_txt_points (const char filename[], vector<struct point3d> &point_cloud) {
    FILE *file = fopen(filename,"r");
    uint32_t np;
    uint32_t k = 0;
    double pos[3];
    fscanf(file,"%u",&np);
    for (uint32_t i = 0; i < np; i++) {
      fscanf(file,"%lf %lf %lf",&pos[0],&pos[1],&pos[2]);
      struct point3d p;
      p.id = k;
      p.pos[0] = pos[0];
      p.pos[1] = pos[1];
      p.pos[2] = pos[2];
      k++;

      point_cloud.push_back(p);
    }
    fclose(file);
}

int main (int argc, char *argv[])
{
    if (argc-1 != 2) {
      printf("-----------------------------------------------------------------------------------------------------------------------------------------\n");
      printf("Usage:> %s <mesh.alg> <points.txt>\n", argv[0]);
      printf("-----------------------------------------------------------------------------------------------------------------------------------------\n");
      printf("<mesh.alg> = Mesh file in ALG format\n");
      printf("<points.txt> = File with the (x,y,z) coordinates of the points to be calculated\n");
      printf("-----------------------------------------------------------------------------------------------------------------------------------------\n");
      exit(EXIT_FAILURE);
    }

    string filename_mesh_alg = argv[1];
    string filename_points_txt = argv[2];

    // ------------------------------------------------------------------------------------------------------------------------------
    // 1) Read the mesh
    vector<struct point3d> mesh_points_alg;
    read_alg_mesh(filename_mesh_alg.c_str(), mesh_points_alg);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    for (uint32_t i = 0; i < mesh_points_alg.size(); i++) {
        points->InsertNextPoint(mesh_points_alg[i].pos[0],mesh_points_alg[i].pos[1],mesh_points_alg[i].pos[2]);
    }
    vtkSmartPointer<vtkUnstructuredGrid> unstructured_grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    unstructured_grid->SetPoints(points);

    // PointLocator (Kd-tree for fast coordinates searching)
    vtkSmartPointer<vtkPointLocator> pointLocator = vtkSmartPointer<vtkPointLocator>::New();
    pointLocator->SetDataSet(unstructured_grid);
    pointLocator->BuildLocator();

    cout << "Mesh number of points = " << mesh_points_alg.size() << endl;

    // 2) Read the points
    vector<struct point3d> point_cloud;
    read_txt_points(filename_points_txt.c_str(),point_cloud);

    
    // 3) Find the closest point in the mesh to the point cloud
    cout << "[!] Finding closest point in the ALG mesh from the TXT file ..." << endl;
    vector<struct point3d> out_mesh_points;
    for (uint32_t i = 0; i < point_cloud.size(); i++) {
      
      double pos[3];
      pos[0] = point_cloud[i].pos[0];
      pos[1] = point_cloud[i].pos[1];
      pos[2] = point_cloud[i].pos[2];
      int id = pointLocator->FindClosestPoint(pos);

      // Get the center position from the ALG cell
      double center[3];
      center[0] = mesh_points_alg[id].pos[0];
      center[1] = mesh_points_alg[id].pos[1];
      center[2] = mesh_points_alg[id].pos[2];

      struct point3d p;
      p.id = i;
      p.pos[0] = center[0];
      p.pos[1] = center[1];
      p.pos[2] = center[2];
      out_mesh_points.push_back(p);
    }

    // 4) Output the closest points
    for (uint32_t i = 0; i < out_mesh_points.size(); i++) {
      printf("%lf %lf %lf\n", out_mesh_points[i].pos[0], out_mesh_points[i].pos[1], out_mesh_points[i].pos[2]);
    }

    return 0;
}
