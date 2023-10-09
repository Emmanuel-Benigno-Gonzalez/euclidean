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

pcl::PCDWriter writer;
std::string directorioPrueba = "../BroccoliFrames/frame_20151104T123858.418992.pcd"; 
std::string directorioEtiquetados = "../BroccoliFrames-gtClusters/gtClusters/frame_20151104T123858.418992.pcd_gt.txt"; 

std::vector<int> guardarClustersEtiquetados();
std::vector<int> guardarIndicesSegmentados(std::vector<pcl::PointIndices> cluster_indices, std::vector<int> indices);
double verificarCoincidenciaIndices(std::vector<int> indicesSegmentados, std::vector<int> indicesEtiquetados, int tam);

int main()
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  reader.read(directorioPrueba, *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl;

  // Aplicar Filtro romoveNAN
  /*Este filtro identifica los indeces donde para ningun valor x,y,z hay un NAN*/
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, indices);
  std::cout << "Cloud: " << cloud->size() << " Indices: " << indices.size() <<std::endl;

  // Algoritmo EuclideanClusterExtraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // Supongamos que tienes un std::vector<int> llamado 'indices' que contiene los índices de los puntos que deseas considerar para la segmentación.
  pcl::IndicesPtr indices_ptr(new std::vector<int>(indices));
  // Configurar la extracción de clústeres con los índices
  ec.setIndices(indices_ptr);
  ec.setClusterTolerance(0.0045);
  ec.setMinClusterSize(500);
  ec.setMaxClusterSize(10000);
  //ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
  
  std::vector<int> indicesEtiquetados = guardarClustersEtiquetados();
  std::vector<int> indicesSegmentados = guardarIndicesSegmentados(cluster_indices, indices);

  // Llamar a la función para verificar coincidencia y obtener el porcentaje
  double porcentajeCoincidencia = verificarCoincidenciaIndices(indicesSegmentados, indicesEtiquetados, cloud->size());
    
  // Mostrar el porcentaje de coincidencia en la consola
  std::cout << "Porcentaje de coincidencia: " << porcentajeCoincidencia << "%" << std::endl;

  // Crear el visualizador
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
  viewer->setBackgroundColor(0, 0, 0); // Fondo gris
  //viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  // Agregar la nube de puntos al visualizador
  viewer->addPointCloud(cloud, "LabelCloud");

  // Configurar el tamaño de los puntos en el visualizador
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "LabelCloud");


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

// Agregar Indices de Segmentados al Vector
std::vector<int> guardarIndicesSegmentados(std::vector<pcl::PointIndices> cluster_indices, std::vector<int> indices){

   //Crear un vector para almacenar todos los índices
  std::vector<int> indicesSegmentados;

  for (const auto &cluster : cluster_indices)
  {
    for (const auto &idx : cluster.indices)
    {
       //std::cout << idx << " "; // Imprimir el índice
       indicesSegmentados.push_back(idx); // Agregar el índice al vector
    }
  }

  return indicesSegmentados;
}

// Extraer los Indices Etiquetados
std::vector<int> guardarClustersEtiquetados()
{

  std::vector<int> terceraColumna;
  std::ifstream file(directorioEtiquetados);

  if (!file.is_open())
  {
    throw std::runtime_error("No se pudo abrir el archivo.");
  }

  std::string line;
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    std::string dummy;
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
double verificarCoincidenciaIndices(std::vector<int> indicesSegmentados, std::vector<int> indicesEtiquetados, int tam)
{
    // Contadores para contar la cantidad de coincidencias
    int coincidencias = 0;
    std::vector<bool> cloud_gt(tam); // Inicializar con valores falsos

    // Marcar los índices etiquetados en el vector cloud_gt como verdaderos
    for (int idxEtiquetado : indicesEtiquetados)
        cloud_gt[idxEtiquetado] = true;

    // Iterar a través de los índices en indicesSegmentados
    for (int idxSegmentado : indicesSegmentados)
        // Buscar el índice en cloud_gt (indicesEtiquetados)
        if (cloud_gt[idxSegmentado])
            coincidencias++;

    std::cout << "Coincidencias: " << coincidencias << std::endl;
    std::cout << "Indices Segmentados: " << indicesSegmentados.size() << std::endl;
    std::cout << "Indices Etiquetados: " << indicesEtiquetados.size() << std::endl;

    // Calcular el porcentaje de coincidencia
    double porcentajeCoincidencia = (static_cast<double>(coincidencias) / indicesEtiquetados.size()) * 100.0;

    return porcentajeCoincidencia;
}
