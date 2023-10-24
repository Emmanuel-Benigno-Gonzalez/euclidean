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
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <vector>
#include <pcl/search/search.h>
#include <cstdlib>
#include <string>
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>

using namespace std;
using namespace pcl;

PCDWriter writer;
string directorioPrueba = "../BroccoliFrames/frame_20151104T123858.418992.pcd"; 
string directorioEtiquetados = "../BroccoliFrames-gtClusters/gtClusters/frame_20151104T123858.418992.pcd_gt.txt"; 

vector<int> guardarClustersEtiquetados();
vector<int> guardarIndicesSegmentados(vector<PointIndices> cluster_indices);
double verificarCoincidenciaIndices(vector<int> indicesSegmentados, vector<int> indicesEtiquetados, int tam);
vector<int> RemoveNAN(PointCloud<PointXYZRGB>::Ptr cloud);
vector<PointIndices> euclideanClusterExtraction(PointCloud<PointXYZRGB>::Ptr cloud, vector<int> indices, double tol, int minTam, int maxTam);
vector<PointIndices> regionGrowing(PointCloud<PointXYZRGB>::Ptr cloud);
vector<int> vectorInt(vector <PointIndices> clusters);
vector<PointIndices> differenceOfNormals(PointCloud<PointXYZRGB>::Ptr cloud);

int main()
{
  // Semilla para la generación de números aleatorios
  srand(static_cast<unsigned int>(time(nullptr)));

  // Read in the cloud data
  PCDReader reader;
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  reader.read(directorioPrueba, *cloud);
  cout << "PointCloud before filtering has: " << cloud->size() << " data points." << endl;

  // Aplicar Filtro romoveNAN
  vector<int> indices = RemoveNAN(cloud);  

  // Algoritmo EuclideanClusterExtraction
  /*double tolerancia = 0.0045;
  int minTamanio = 500;
  int maxTamanio = 10000;
  vector<PointIndices> clusters = euclideanClusterExtraction(cloud, indices, tolerancia, minTamanio, maxTamanio);
  vector<int> indicesSegmentados = guardarIndicesSegmentados(clusters);*/

  //Difference Of Normals
  vector<PointIndices> clusters = differenceOfNormals(cloud); 
  vector<int> indicesSegmentados = guardarIndicesSegmentados(clusters);

  // Algoritmo Plane model segmentation
  //vector <PointIndices> clusters = regionGrowing(cloud);
  //vector <int> indicesSegmentados = vectorInt(clusters);
  
  vector<int> indicesEtiquetados = guardarClustersEtiquetados();
  
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
  
  for (const auto& cluster : clusters) {
    int colorR = rand() % 256;
    int colorG = rand() % 256;
    int colorB = rand() % 256;

    for (const auto i:cluster.indices){
      if(find(indicesEtiquetados.begin(), indicesEtiquetados.end(), i) != indicesEtiquetados.end()){
        cloud->points[i].r = colorR;
        cloud->points[i].g = colorG;
        cloud->points[i].b = colorB;
      }
    }  
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
vector<int> guardarIndicesSegmentados(vector<PointIndices> cluster_indices){

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

vector <PointIndices> regionGrowing(PointCloud<PointXYZRGB>::Ptr cloud){

  search::Search<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
  PointCloud <Normal>::Ptr normals (new PointCloud <Normal>);
  NormalEstimation<PointXYZRGB, Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  IndicesPtr indices (new vector <int>);
  removeNaNFromPointCloud(*cloud, *indices);

  RegionGrowing<PointXYZRGB, Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (70);
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (23 / 180.0 * M_PI);
  //reg.setCurvatureThreshold (1.0);

  vector <PointIndices> clusters;
  reg.extract (clusters);

  return clusters;
}

vector<int> vectorInt(vector <PointIndices> clusters){

  vector<int> segmentedIndices; // Vector de índices segmentados

  for (const auto& cluster : clusters) {
    vector<int> clusterIndices = cluster.indices;
    segmentedIndices.insert(segmentedIndices.end(), clusterIndices.begin(), clusterIndices.end());
  }

  return segmentedIndices;
}

vector<PointIndices> differenceOfNormals(PointCloud<PointXYZRGB>::Ptr cloud){
  double scale1 = 0.1;
  double scale2 = 0.2;
  double threshold = 0.1;
  double segradius = 0.0045;

  search::Search<PointXYZRGB>::Ptr tree;
  if (cloud->isOrganized()) {
    tree.reset(new search::OrganizedNeighbor<PointXYZRGB>());
  } else {
    tree.reset(new search::KdTree<PointXYZRGB>(false));
  }

  tree->setInputCloud(cloud);

  if (scale1 >= scale2) {
    cerr << "Error: Large scale must be > small scale!" << endl;
    exit(EXIT_FAILURE);
  }

  NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setViewPoint(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());

  cout << "Calculating normals for scale..." << scale1 << endl;
  PointCloud<PointNormal>::Ptr normals_small_scale(new PointCloud<PointNormal>);
  ne.setRadiusSearch(scale1);
  ne.compute(*normals_small_scale);

  cout << "Calculating normals for scale..." << scale2 << endl;
  PointCloud<PointNormal>::Ptr normals_large_scale(new PointCloud<PointNormal>);
  ne.setRadiusSearch(scale2);
  ne.compute(*normals_large_scale);

  PointCloud<PointNormal>::Ptr doncloud(new PointCloud<PointNormal>);
  copyPointCloud(*cloud, *doncloud);

  cout << "Calculating DoN... " << endl;
  DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
  don.setInputCloud(cloud);
  don.setNormalScaleLarge(normals_large_scale);
  don.setNormalScaleSmall(normals_small_scale);

  if (!don.initCompute()) {
    cerr << "Error: Could not initialize DoN feature operator" << endl;
    exit(EXIT_FAILURE);
  }

  don.computeFeature(*doncloud);

  PCDWriter writer;
  writer.write<PointNormal>("don.pcd", *doncloud, false);

  cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

  ConditionOr<PointNormal>::Ptr range_cond(
      new ConditionOr<PointNormal>());

  range_cond->addComparison(FieldComparison<PointNormal>::ConstPtr(
      new FieldComparison<PointNormal>("curvature", ComparisonOps::GT, threshold)));

  ConditionalRemoval<PointNormal> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(doncloud);

  PointCloud<PointNormal>::Ptr doncloud_filtered(new PointCloud<PointNormal>);

  condrem.filter(*doncloud);

  doncloud = doncloud_filtered;

  search::KdTree<PointNormal>::Ptr segtree(new search::KdTree<PointNormal>);
  segtree->setInputCloud(doncloud);

  vector<PointIndices> cluster_indices;
  EuclideanClusterExtraction<PointNormal> ec;

  ec.setClusterTolerance(segradius);
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(100000);
  ec.setSearchMethod(segtree);
  ec.setInputCloud(doncloud);
  ec.extract(cluster_indices);

  
  return cluster_indices;
}