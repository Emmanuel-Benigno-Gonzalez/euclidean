#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h> //libreria para usar PointXYZRGB
#include <pcl/io/pcd_io.h>   //libreria para usar PCDReader
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h> //libreria para filtro que remueve valores NaN
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> // for setw, 
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;


PCDWriter writer;
string directorioPrueba = "../BroccoliFrames/frame_20151104T123858.418992.pcd"; 
string directorioEtiquetados = "../BroccoliFrames-gtClusters/gtClusters/frame_20151104T123858.418992.pcd_gt.txt"; 

vector<int> guardarClustersEtiquetados();
vector<int> guardarIndicesSegmentados(vector<PointIndices> cluster_indices, vector<int> indices);
double verificarCoincidenciaIndices(vector<int> indicesSegmentados, vector<int> indicesEtiquetados, int tam);
vector<int> RemoveNAN(PointCloud<PointXYZRGB>::Ptr cloud);
vector<PointIndices> euclideanClusterExtraction(PointCloud<PointXYZRGB>::Ptr cloud, vector<int> indices, double tol, int minTam, int maxTam);

int main()
{
  // Read in the cloud data
  PCDReader reader;
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  reader.read(directorioPrueba, *cloud);
  cout << "PointCloud before filtering has: " << cloud->size() << " data points." << endl;

  // Aplicar Filtro romoveNAN
  vector<int> indices = RemoveNAN(cloud);  

  // Algoritmo EuclideanClusterExtraction
  double tolerancia = 0.0045;
  int minTamanio = 500;
  int maxTamanio = 10000;
  vector<PointIndices> cluster_indices = euclideanClusterExtraction(cloud, indices, tolerancia, minTamanio, maxTamanio);

  // Algoritmo Plane model segmentation
  
  
  vector<int> indicesEtiquetados = guardarClustersEtiquetados();
  vector<int> indicesSegmentados = guardarIndicesSegmentados(cluster_indices, indices);

  // Llamar a la función para verificar coincidencia y obtener el porcentaje
  double porcentajeCoincidencia = verificarCoincidenciaIndices(indicesSegmentados, indicesEtiquetados, cloud->size());
    
  // Mostrar el porcentaje de coincidencia en la consola
  cout << "Porcentaje de coincidencia: " << porcentajeCoincidencia << "%" << endl;

  // Crear el visualizador
  visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("PointCloud Viewer"));
  viewer->setBackgroundColor(0, 0, 0); // Fondo gris
  //viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  // Agregar la nube de puntos al visualizador
  viewer->addPointCloud(cloud, "LabelCloud");

  // Configurar el tamaño de los puntos en el visualizador
  viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, "LabelCloud");


  for (const auto i:indicesEtiquetados){
    cloud->points[i].r = 255;
    cloud->points[i].g = 0;
    cloud->points[i].b = 0;
  }
  
  for (const auto i:indicesSegmentados){
    cloud->points[i].r = 0;
    cloud->points[i].g = 255;
    cloud->points[i].b = 0;
  }

  viewer->removeAllPointClouds();
  viewer->addPointCloud(cloud, "labelCloud");

  // Mostrar la nube de puntos y esperar hasta que el usuario cierre la ventana
  viewer->spin();
    
  return (0);
}

//Remover los NAN de la Nube de Puntos 
vector<int> RemoveNAN(PointCloud<PointXYZRGB>::Ptr cloud){
  /*Este filtro identifica los indeces donde para ningun valor x,y,z hay un NAN*/
  vector<int> indices;
  removeNaNFromPointCloud(*cloud, indices);
  cout <<"Indices: " << indices.size() <<endl;
  return indices;
}

//Euclidean Cluster Extraction
vector<PointIndices> euclideanClusterExtraction(PointCloud<PointXYZRGB>::Ptr cloud, vector<int> indices, double tol, int minTam, int maxTam){

  vector<PointIndices> cluster_indices;
  EuclideanClusterExtraction<PointXYZRGB> ec;
  // Configurar la extracción de clústeres con los índices
  IndicesPtr indices_ptr(new vector<int>(indices));
  ec.setIndices(indices_ptr);
  ec.setClusterTolerance(tol);
  ec.setMinClusterSize(minTam);
  ec.setMaxClusterSize(maxTam);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  return cluster_indices;
}

// Agregar Indices de Segmentados al Vector
vector<int> guardarIndicesSegmentados(vector<PointIndices> cluster_indices, vector<int> indices){

   //Crear un vector para almacenar todos los índices
  vector<int> indicesSegmentados;

  for (const auto &cluster : cluster_indices)
  {
    for (const auto &idx : cluster.indices)
    {
       //cout << idx << " "; // Imprimir el índice
       indicesSegmentados.push_back(idx); // Agregar el índice al vector
    }
  }

  return indicesSegmentados;
}

// Extraer los Indices Etiquetados
vector<int> guardarClustersEtiquetados()
{

  vector<int> terceraColumna;
  ifstream file(directorioEtiquetados);

  if (!file.is_open())
  {
    throw runtime_error("No se pudo abrir el archivo.");
  }

  string line;
  while (getline(file, line))
  {
    istringstream iss(line);
    string dummy;
    int valorSegCol, valorTerCol;

    // Leer las tres columnas, pero solo almacenar la tercera
    iss >> dummy >> valorSegCol >> valorTerCol;

    //Agregar los indices a terceraColumna
    terceraColumna.push_back(valorTerCol);
  }

  file.close();
  
  return terceraColumna;
}

// Función para verificar la coincidencia de índices y calcular el porcentaje
double verificarCoincidenciaIndices(vector<int> indicesSegmentados, vector<int> indicesEtiquetados, int tam)
{
    // Contadores para contar la cantidad de coincidencias
    int coincidencias = 0;
    vector<bool> cloud_gt(tam); // Inicializar con valores falsos

    // Marcar los índices etiquetados en el vector cloud_gt como verdaderos
    for (int idxEtiquetado : indicesEtiquetados)
        cloud_gt[idxEtiquetado] = true;

    // Iterar a través de los índices en indicesSegmentados
    for (int idxSegmentado : indicesSegmentados)
        // Buscar el índice en cloud_gt (indicesEtiquetados)
        if (cloud_gt[idxSegmentado])
            coincidencias++;

    cout << "Coincidencias: " << coincidencias << endl;
    cout << "Indices Segmentados: " << indicesSegmentados.size() << endl;
    cout << "Indices Etiquetados: " << indicesEtiquetados.size() << endl;

    // Calcular el porcentaje de coincidencia
    double porcentajeCoincidencia = (static_cast<double>(coincidencias) / indicesEtiquetados.size()) * 100.0;

    return porcentajeCoincidencia;
}
