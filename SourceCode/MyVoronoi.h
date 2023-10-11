#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/polygon/voronoi.hpp>
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

#include <time.h>      
#include <omp.h> // OpenMP for multithreading
#include "rng.h"
#include <random>
#include <unordered_map>

#include <QDebug>

//#include "TempGrid.h"
#include "Rainbow.h"
#include "TempGridFDE.h"
#include "GenDensityMap.h"
#include "DevidedVoronoi.h"
#include "HVD.h"
#include "HSSPathFinding.h"


//class Vvertex
//{
//private:
//	int index;
//	const boost::polygon::voronoi_vertex<double>* vertex;
//
//public:
//	Vvertex() : vertex(0) {}
//	~Vvertex() { if (vertex != NULL) delete vertex; }
//	void Initialize() {
//
//	}
//};

//class CrackEdge  //One Single Branch
//{
//private:
//	std::vector<const boost::polygon::voronoi_vertex<double>*> OneBranch;
//public:
//	void AddPoint(const boost::polygon::voronoi_vertex<double>* vert) { OneBranch.push_back(vert); }
//	int GetSize() { return OneBranch.size(); }
//	const boost::polygon::voronoi_vertex<double>* GetPoint(int index) { return OneBranch[index]; }
//};

class Node
{
public:
	Node(int time, HVD::vert* vertex, double potential)
	{
		parent = NULL;
		T = time;
		growth_potential = potential;
		voronoi_vertex = vertex;
		m_pos = cv::Point2d(vertex->m_pos.x, vertex->m_pos.y);
		direction = -1;  //only root node do not have direction, the direction of the branch depends on the first children
		perceive_radius = 0;
	}
	Node(int time, HVD::vert* vertex, double potential ,int direction_index)
	{
		parent = NULL;
		T = time;
		growth_potential = potential;
		voronoi_vertex = vertex;
		m_pos = cv::Point2d(vertex->m_pos.x, vertex->m_pos.y);
		direction = direction_index;  //the direction of the branch depends on the first children
		perceive_radius = 0;
	}
	Node(int time, cv::Point2d pos, double potential)
	{
		parent = NULL;
		T = time;
		growth_potential = potential;
		voronoi_vertex = NULL;
		m_pos = pos;
		direction = -1;  //the direction of the branch depends on the first children
		perceive_radius = 0;
	}
	~Node()
	{
		if (parent != NULL) delete parent;
		parent = NULL;
	}
private:
	std::vector<Node*> children;
	Node* parent;
	int T;
	HVD::vert* voronoi_vertex;
	int direction; //the child's index following the growing direction 

	double growth_potential;  //the energy
	double real_direction;  //the true direction tendency angle indicated by the temperature field
	double perceive_radius;  // the perceive radius for detecting other crack

public:
	double m_width = -1.0;
	double m_depth = -1.0;
	vec2 normal;
	cv::Point2d m_pos;

public:
	std::vector<Node*> Return_Children() { return children; }
	int Return_Year() {return T;}
	HVD::vert* Return_Vertex() { return voronoi_vertex; }
	Node* ReturnParent() { return parent; }
	void SetParent(Node* node) { parent = node; }
	void AppendChild(Node* node) { children.push_back(node); }
	int GetDirection() { return direction; }
	double ReturnPotential() { return growth_potential;}
	void SetRadius(double r) { perceive_radius = r; }
	double ReturnRadius() { return perceive_radius; }
	
};

struct QuadNode
{
public:
	QuadNode(double x, double y, double normalized_W, double normalized_H, double normalized_X)
	{
		m_x = x;
		m_y = y;
		m_normalized_W = normalized_W;
		m_normalized_H = normalized_H;
		m_normalized_X = normalized_X;
	}

public:
	double m_x;
	double m_y;
	double m_normalized_W;
	double m_normalized_H;
	double m_normalized_X;
};

class MyVoronoi
{
using Site = cv::Point2i;

private:	
	int m_row;  // y direction hangshu
	int m_column; // x direction lieshu
	int m_num_points;
	int m_SiteDense; //only for the map loading
	
	HVD m_Voronoi_Hierarchy;
	voronoi_diagram<double> m_vd;  // The voronoi map
	voronoi_diagram<double> m_Voronoi_Damage_Diagram;  // The voronoi map with damage points
	cv::Mat m_vmap;              // The rendered crack
	cv::Mat m_rendermap;         // The rendered crack effect (blue-red map)
	
	std::vector<Site> m_sites;  // The cell's core
	std::vector<cv::Point> m_sites_with_Damage;  // new cell's cores
	std::vector<double> m_sites_alpha; //Saved the temperature data of the FIRST LEVEL voronoi map (!DO NOT INCLUDE NEW DAMAGE POINTS)

	//Those two are one2one sets, ie: their index are the same
	//std::vector<CrackEdge*> m_crackset;  // The sets of braches
	std::vector<Node*> m_crackseeds;  // The sets of braches initial points <=> ID
	int m_scndcrackseedindex;  //save the start index of the secondary crack seeds
	bool m_ifdrawscndcrack;

	std::vector<voronoi_diagram<double>::const_cell_iterator> m_damageseeds;  // Recording the voronoi cell's index
	std::vector<Node*> m_damagecrackseeds;

	//std::unordered_map<const boost::polygon::voronoi_vertex<double>*, int> m_garbage_vets;
	std::unordered_map<int, int> m_garbage_vets;  //first vertex index, second bool

	int m_old_time = 0; // to judge the time is increase or decrease
	//std::vector < const boost::polygon::voronoi_vertex<double>*> m_now_vertex; //the end of each branch
	double m_destroy_potential = 1;  // when the energy <= 1; destroy

	//TempGrid* m_Temp_Grid;
	FDE_Topology* m_FDE_Mesh;

	std::vector<vec3> m_Damage_Points;

public:
	double m_max_width = -1.0;
	double m_max_depth = -1.0;
	double m_min_depth = 100000;
	double m_min_width = 100000;
	std::vector<std::vector<QuadNode>> m_quad_garage;

private:
	// no use std::unordered_map<int, int> m_DicSiteandCell;
	GenDensityMap* m_DensityMap;
	std::vector<std::vector<SubVD>> m_SubdevideHierarchy;  //sub index is parent's vd cell's index
	std::vector< voronoi_diagram<double> > m_VdGarage;	//save all the local vds for different level besides the density map (first) level
	std::vector< std::vector<Site>> m_SiteGarage; //save all the local sites for different level besides the density map (first) level

	int m_level = 2;

	
	//std::vector<std::unordered_map<int, int>> m_DicKidSitetoParentCell; //for the 3 levels
	
	//std::unordered_map<int, int> m_DicKidCelltoParentCell;
	//
	//std::unordered_map<int, int> m_DicEdgeInorOut; //first small vor edge index, second 0-inside x-big vor edge
	//std::unordered_map<int, std::unordered_map<int, int>> m_DicCellstoEdge;  //two cell->common edge
	//std::unordered_map<int, std::vector<int>> m_Dic_Boundary_relation; //first edge index in parent vd, second unfordered children gloabal edge indices

public:
	int nsites();
	int nfpoints();

public:
	MyVoronoi(int n_points, int rows, int cols, int seed = -1, bool regularize = true);
	~MyVoronoi();

public:
	
	void SetSites(int n_points, int rows, int cols, int seed = -1, bool regularize = true);
	void Construct_VoronoiMap();

	void SaperateDensityMap(GenDensityMap* DensityMap, float GlobalScellDensity);
	void SaperateSubVorMap(voronoi_diagram<double>* vd, std::vector<cv::Point2i>* sites, int Density);
	void ConstructTopo();  //construct parent and kid relationship and updated site and alpha information
	void SmoothHVDedges(double theta);
	std::vector<std::pair<std::vector<cv::Point2i>, std::vector<cv::Point3f>>> DrawTopo();

	void InsertDamagetoVoronoi();
	void SetDamagePoint(vec3 P);
	void ShowDamage();
	void ShowDamageWithVoronoi();
	vec2 ClearLastDamagePoint();
	void ClearAllDamagePoint();

	
	void CreateCrack();  //Add a new brach
	bool CheckMergeDamageHole(Node* node);  //for everycrack checking the damage hole

//-------------------------Temperature information input------------------------------------
	void SetTempVectorField(FDE_Topology* T);

//-------------------------Happends after the temperature has been set-------------------------
	void IncreaseBranch(int erostime = 60.0); //Increase a branch

//----------------------------------------------
	double Uniformlength(double l);
	void SetSecondStartOnPriEdge();
	void TestDrawPriandSecEdge();
	void ArrangeSecondaryCrack();

	void RenderCrack(int erostime, Node* node);
	bool AddChildren(Node* node);
	void RenderDamageCrack(Node* node);

	void ClearMap();
	void EraseData();

	void DrawLine(Node* node1, Node* node2, int line_level = -1);
	void DrawLine(Site p1, Site p2, cv::Mat m, int width);

	void SetDrawFlag(bool b);
	void DrawCrack();
	void DrawQuad(Node* node1, Node* node2);
//--------------------------Functional functions-----------------
	//void ChangePicIntoPixel(QString s, int Resoution);//Voronoi Distribution with gray scale picture
	cv::Mat ShowMap();
	void DrawRuler(cv::Mat m);
	//double CalculateArea(const voronoi_diagram<double>::cell_type* cell);	
	double CalculateArea(HVD::cell* cell);

	void CalHomogeneous(); //For getting the statistical information of the topology for homogeneous
	void CalDamHomogeneous();

	vec2 ReturnDim() { return vec2(m_row, m_column); }
	std::vector<double>* ReturnAlphaList() { return &m_sites_alpha; }
	voronoi_diagram<double>* ReturnVorDiagramWithDamage() { return &m_Voronoi_Damage_Diagram; }
	std::vector<vec3> ReturnDamagePoints() { return m_Damage_Points; }
	HVD* ReturnHVD() { return &m_Voronoi_Hierarchy; }
};