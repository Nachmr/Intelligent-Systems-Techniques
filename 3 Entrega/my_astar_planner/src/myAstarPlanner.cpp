/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  Copyright (c) 2015, Juan Fdez-Olivares
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Juan Fdez-Olivares, Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include "../include/my_astar_planner/myAstarPlanner.h"
#include <pluginlib/class_list_macros.h>

// para debugging
#include <sstream>
#include <string>







//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)

namespace myastar_planner {

  //devuelve un puntero a un nodo en una lista de nodos (nodo = coupleOfCells) a partir del índice del nodo
  list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID);

  //comprueba si un índice de nodo existe en una lista de nodos.
  bool isContains(list<coupleOfCells> & list1, int cellID);

  MyastarPlanner::MyastarPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  MyastarPlanner::MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  //inicializador del global_planner, mejor no tocar nada.
  void MyastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);

      //vamos a asumir estos parámetros, que no es necesario enviar desde el launch.
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      //world_model_ = new base_local_planner::CostmapModel(*costmap_);

      //el plan se va a publicar en el topic "planTotal"
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("planTotal",1);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //esta función puede usarse para ayudar a calcular rutas seguras
  //está preparada para obtener el footprint del robot y devolver un valor representando el coste de la posición del robot.
  double MyastarPlanner::footprintCost(double x_i, double y_i, double theta_i){
    unsigned int robot_mx, robot_my, cell_mx, cell_my;

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    //Obtiene un vector de puntos, digamos que son los puntos que unen el poligono donde que "envuelve" al robot
    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    double seguridad = 0.0;
    std::vector<unsigned int> celdas_robot;
    //Un polígono inscrito en un circulo --> si todos sus vértices están contenidos en ella.
    // Si l = sqrt(r^2 + r^2) --> l/sqrt(2) = radio
    double lado = sqrt((pow(footprint[0].x - footprint[1].x,2))+pow(footprint[0].y - footprint[1].y, 2));
    double radio = lado/(sqrt(2)) * theta_i;// Theta porcentaje de reduccion en radio

    //Aqui deberia comprobarse que ningun punto del contorno del robot que se encuentra en footprint
    //no se encuentre con ningun obstaculo
    //Hay que tener en cuenta dos areas, en caso de que escojamos un circulo, aunque tambien vale el mismo poligono
    //que forma el robot: http://www.vitutor.net/2/1/3.html
    //1.La cirscunscrita --> que es como un circulo interno en el robot, por tanto, tendria que coincidir con
    //las coordendas del robot para que no acepte una casilla con obstaculo
    //2.La inscrita --> que tiene en cuenta el pico", el punto de la parte delantera
    //del robot para trazar el area del circulo que se supone que no deberia de haber ningun obstaculo, es decir,
    //cuantas celdas se van a dejar de seguridad para que pueda pasar el robot.
    //Es una forma de calcular la seguridad.
    //Otra forma, seria ver que no este ningun punto, pero habria que tener en cuenta la union de esos puntos
    //para que no diera error...

    //ROS_INFO("PosicionRobot ");
    //El siguiente for muestra el vector de los 5 puntos. Teniendo en cuenta que tiene forma cuadrada con los 4
    //puntos y con el ultimo punto que indica "la cabeza"/direccion del robot.
    // los cuales se muestran en el siguiente orden, teniendo como referencia de adelante la cabeza:
    //1. el punto que esta a la derecha y detras.
    //2. el punto de detras y a la izuierda.
    //3. el punto de delante e izquierda.
    //4. Cabeza
    //5. el punto de delante y derecha.
    for(int i=0; i<footprint.size(); i++)
    {
        //Pasar las coordenadas del mundo a coordenadas en el mapa, para el robot
        costmap_->worldToMap(footprint[i].x, footprint[i].y, robot_mx, robot_my);

        //ROS_INFO("PosicionRobot");
        //ROS_INFO("Punto world %d: x,y: %.2f, %.2f", i, footprint[i].x, footprint[i].y);
        //ROS_INFO("Punto cell %d: x,y: %d, %d", i, robot_mx, robot_mx);

        //Como son celdas x e y, lo que devuelve la anterior funcion, se suponen que son la misma celda
        celdas_robot.push_back(robot_mx);
    }

    //Pasar las coordenadas del mundo a coordenadas en el mapa, para la posicion vecina
    //costmap_->worldToMap(x_i, y_i, cell_mx, cell_my);
    //ROS_INFO("PosicionVecina");
    //ROS_INFO("Punto world %d: x,y: %.2f, %.2f", i, x_i, y_i);
    //ROS_INFO("Punto cell %d: x,y: %d, %d", i, cell_mx, cell_my);

    //Si el radio es cero, se realizara el poligono circunscrito al circulo --> circulo dentro del poligono
    if(radio==0)
        radio = lado/2;
    //Se comprueba que las casillas adyacentes al vecino no sean un obstaculo
    for (int x=-1;x<=1;x++)
    {
        for (int y=-1; y<=1;y++){
            //Pasar las coordenadas del mundo a coordenadas en el mapa, para la posicion vecina
            costmap_->worldToMap(x_i+radio*x, y_i+radio*y, cell_mx, cell_my);
            //ROS_INFO("Punto cell: x,y: %d, %d", cell_mx, cell_my);
            //check whether the index is valid
            if ((cell_mx>=0)&&(cell_mx < costmap_->getSizeInCellsX())&&(cell_my >=0 )&&(cell_my < costmap_->getSizeInCellsY()))
            {
                //Si hay un obstaculo en la casilla
                if(costmap_->getCost(cell_my,cell_my) == costmap_2d::LETHAL_OBSTACLE   && (!(x==0 && y==0))){
                  seguridad += 0.5;
                }//Si hay un obstaculo en parte de la celda... ??
                else if(costmap_->getCost(cell_my,cell_my) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE   && (!(x==0 && y==0))){
                  seguridad += 0.2;
                }
                else if(costmap_->getCost(cell_my,cell_my) == costmap_2d::FREE_SPACE   && (!(x==0 && y==0))){
                  seguridad -= 0.1;
                }
            }
        }
    }

    return seguridad;
  }

  //función llamada por move_base para obtener el plan.
  bool MyastarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    //***********************************************************
    // Inicio de gestion de ROS, mejor no tocar nada en esta parte
    //***********************************************************
    if(!initialized_){
      ROS_ERROR("The astar planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_INFO("MyastarPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();

    //obtenemos el costmap global  que está publicado por move_base.
    costmap_ = costmap_ros_->getCostmap();


    //Obligamos a que el marco de coordenadas del goal enviado y del costmap sea el mismo.
    //esto es importante para evitar errores de transformaciones de coordenadas.
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    //obtenemos la orientación start y goal en start_yaw y goal_yaw.
    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);


    /**************************************************************************/
    /*************** HASTA AQUÍ GESTIÓN DE ROS *********************************/
    /****************************************************************************/

    //pasamos el goal y start a un nodo (estructura coupleOfCells)
    coupleOfCells cpstart, cpgoal;
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    unsigned int mgoal_x, mgoal_y;
    costmap_->worldToMap(goal_x,goal_y,mgoal_x, mgoal_y);
    cpgoal.index = MyastarPlanner::costmap_->getIndex(mgoal_x, mgoal_y);
    cpgoal.parent=0;
    cpgoal.gCost=0;
    cpgoal.hCost=0;
    cpgoal.fCost=0;

    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    unsigned int mstart_x, mstart_y;
    costmap_->worldToMap(start_x,start_y, mstart_x, mstart_y);
    cpstart.index = MyastarPlanner::costmap_->getIndex(mstart_x, mstart_y);
    cpstart.parent =cpstart.index;
    cpstart.gCost = 0;
    cpstart.hCost = MyastarPlanner::calculateHCost(cpstart.index,cpgoal.index);

    //insertamos el nodo inicial en abiertos
    MyastarPlanner::openList.push_back(cpstart);


    ROS_INFO("Inserto en Abiertos: %d", cpstart.index );
    ROS_INFO("Index del goal: %d", cpgoal.index );

    unsigned int explorados = 0;
    unsigned int currentIndex = cpstart.index;

    while (!MyastarPlanner::openList.empty() )//&& explorados < 3) //while the open list is not empty continuie the search
    {

        //escoger el nodo (coupleOfCells) de abiertos que tiene el valor más pequeño de f.
        list<coupleOfCells>::iterator ite = openList.begin();
        double fmin = (*ite).fCost;

        coupleOfCells COfCells = (*ite);

        while(ite != openList.end()){
            if(fmin > (*ite).fCost){
                fmin = (*ite).fCost;
                COfCells = (*ite);
            }
            ite++;
        }

        currentIndex=COfCells.index;

        //vamos a insertar ese nodo en cerrados

            //obtenemos un iterador a ese nodo en la lista de abiertos
            list<coupleOfCells>::iterator it=getPositionInList(openList,currentIndex);


            //copiamos el contenido de ese nodo a una variable nodo auxiliar
            cpstart.index=currentIndex;
            cpstart.parent=(*it).parent;
            cpstart.gCost=(*it).gCost;
            cpstart.hCost=(*it).hCost;
            cpstart.fCost=(*it).fCost;


        //y esa variable la insertamos en cerrados
            MyastarPlanner::closedList.push_back(cpstart);
        //ROS_INFO("Inserto en CERRADOS: %d", (*it).index );
           ROS_INFO("G: %f, H: %f, F: %f", (*it).gCost, (*it).hCost, (*it).fCost);
           ROS_INFO("Index: %d Parent: %d", (*it).index, (*it).parent);


          // Si el nodo recién insertado es el goal, ¡plan encontrado!

          if(currentIndex==cpgoal.index)
          {
            //el plan lo construimos partiendo del goal, del parent del goal y saltando en cerrados "de parent en parent"
            //y vamos insertando al final los waypoints (los nodos de cerrados)

              ROS_INFO("PLAN ENCONTRADO!!!");

            //convertimos goal a poseStamped nueva


              geometry_msgs::PoseStamped pose;
              pose.header.stamp =  ros::Time::now();
              pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el goal pasado por parámetro
              pose.pose.position.x = goal_x;
              pose.pose.position.y = goal_y;
              pose.pose.position.z = 0.0;
              pose.pose.orientation.x = 0.0;
              pose.pose.orientation.y = 0.0;
              pose.pose.orientation.z = 0.0;
              pose.pose.orientation.w = 1.0;

              //lo añadimos al plan%
              plan.push_back(pose);
              ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);

              coupleOfCells currentCouple = cpstart;
              unsigned int currentParent = cpstart.parent;

              while (currentCouple.index != currentParent) //e.d. mientras no lleguemos al nodo start
              {
                //encontramos la posición de currentParent en cerrados

                 list<coupleOfCells>::iterator it=getPositionInList(closedList,currentParent);
                //hacemos esa posición que sea el currentCouple
                currentCouple.index=currentParent;
                currentCouple.parent=(*it).parent;
                currentCouple.gCost=(*it).gCost;
                currentCouple.hCost=(*it).hCost;
                currentCouple.fCost=(*it).fCost;

                //creamos una PoseStamped con la información de currentCouple.index

                        //primero hay que convertir el currentCouple.index a world coordinates
                unsigned int mpose_x, mpose_y;
                double wpose_x, wpose_y;

                costmap_->indexToCells(currentCouple.index, mpose_x, mpose_y);
                costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);


                        //después creamos la pose
                geometry_msgs::PoseStamped pose;
                pose.header.stamp =  ros::Time::now();
                pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el de la entrada
                pose.pose.position.x = wpose_x;
                pose.pose.position.y = wpose_y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;

                //insertamos la pose en el plan
                plan.push_back(pose);
                ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);
                //hacemos que currentParent sea el parent de currentCouple
                currentParent = currentCouple.parent;
              }

            ROS_INFO("Sale del bucle de generación del plan.");
            std::reverse(plan.begin(),plan.end());

            //lo publica en el topic "planTotal"
            publishPlan(plan);
            return true;
          }

          //Si no hemos encontrado plan aún eliminamos el nodo insertado de ABIERTOS.
            bool esta = false;
            ite = openList.begin();
            while(ite != openList.end()){
            if ((*ite).index == COfCells.index){
                for(list<coupleOfCells>::iterator itclos = closedList.begin() ; itclos != closedList.end(); itclos++){
                    if((*itclos).index == (*ite).index){
                        esta = true;
                        break;
                    }
                }
                if(!esta)
                    closedList.push_back((*ite));
                openList.erase(ite);
                break;
            }
                ite++;
            }

          //Buscamos en el costmap las celdas adyacentes a la actual
          vector <unsigned int> neighborCells=findFreeNeighborCell(currentIndex);
          int i=0, total=neighborCells.size();
          //Ignoramos las celdas que ya existen en CERRADOS
          //ROS_INFO("Vecinos encontrados: %d", neighborCells.size());
          while(i<total)
          {
            for(list<coupleOfCells>::iterator itclos = closedList.begin() ; itclos != closedList.end(); itclos++){
                if(neighborCells[i] == (*itclos).index){
                    neighborCells.erase(neighborCells.begin()+i);
                    total--;
                    i--;
                    break;
                }
            }
            i++;
          }
          //ROS_INFO("Vecinos que no estan en cerrados: %d", neighborCells.size());

          //Determinamos las celdas que ya están en ABIERTOS y las que no están en ABIERTOS
          total=neighborCells.size();
          i=0;
          while(i<total)
          {
            for(list<coupleOfCells>::iterator itopen = openList.begin() ; itopen != openList.end(); itopen++){
                if(neighborCells.at(i) == (*itopen).index){
                    neighborCells.erase(neighborCells.begin()+i);
                    total--;
                    i--;
                    break;
                }
            }
            i++;
          }
          //ROS_INFO("Vecinos que no estan en abiertos: %d", neighborCells.size());

         //Añadimos a ABIERTOS las celdas que todavía no están en ABIERTO, marcando el nodo actual como su padre
         addNeighborCellsToOpenList(openList, neighborCells, currentIndex, cpstart.gCost, cpgoal.index);
         explorados++;

         //Para los nodos que ya están en abiertos, comprobar en cerrados su coste y actualizarlo si fuera necesario
         list<coupleOfCells>::iterator itclos = closedList.begin();
         for(list<coupleOfCells>::iterator itopen = openList.begin() ; itopen != openList.end(); itopen++){
            while(itclos != closedList.end()){
                //Lista de cerrados, es el arbol de evaluación
                //Lista de abiertos, son los nodos adyacentes de la posición a evaluar para saber el siguiente
                //Con lo cual, hay que comparar cada nodo abierto que este en cerrado y actualizar en el caso de que
                //el abierto sea mejor.
                if ((*itopen).parent == (*itclos).parent)
                {
                    //ROS_INFO("Abiertos: %d", (*itopen).index);
                    //ROS_INFO("Cerrados: %d", (*itclos).index);
                    //ROS_INFO("Padre: %d", (*itopen).parent);
                    //Si es mejor el costo evaluado modificar la lista de cerrados
                	if((*itclos).gCost>(*itopen).gCost)
                	{
                		/*(*itclos).gCost = (*itopen).gCost;
                		(*itclos).fCost = (*itclos).gCost + (*itclos).hCost;
                        (*itclos).parent = (*itopen).parent;*/
                        //Al borrar apunta al siguiente de la lista
                        itclos = closedList.erase(itclos);
                        closedList.push_back((*itopen));
                        itclos--;
                	}
                }
                itclos++;
            }
         }
    }


    /*CP.gCost=gCostParent+getMoveCost(parent,neighborCells[i]);

    //calculate the hCost: Euclidian distance from the neighbor cell to the goalCell
    CP.hCost=calculateHCost(neighborCells[i],goalCell);
    //calculate fcost

    CP.fCost=CP.gCost+CP.hCost;*/

    if(openList.empty())  // if the openList is empty: then failure to find a path
        {
            ROS_INFO("Failure to find a path !");
            return false;
           // exit(1);
        }

};


//calculamos H como la distancia euclídea hasta el goal
double MyastarPlanner::calculateHCost(unsigned int start, unsigned int goal) {
  unsigned int mstart_x, mstart_y, mgoal_x, mgoal_y;
  double wstart_x, wstart_y, wgoal_x, wgoal_y;

  //trasformamos el indice de celdas a coordenadas del mundo.
  //ver http://docs.ros.org/indigo/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

  //Aqui lo que hace es transformar el indice start en coordenadas de celda donde las guarda en mstart_x/y
  costmap_->indexToCells(start, mstart_x, mstart_y);
  //Para luego transformar esas coordenadas de celda en coordenadas del mundo
  costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
  //Igual para el objetivo y así poder calcular la distancia euclidea y saber cuanto de lejos esta del objetivo
  costmap_->indexToCells(goal, mgoal_x, mgoal_y);
  costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);

  //H --> se trata de una estimacion
  return sqrt((pow(wstart_x - wgoal_x,2))+pow(wstart_y - wgoal_y, 2));
 }


//comparamos F para dos nodos.
bool MyastarPlanner::compareFCost(coupleOfCells const &c1, coupleOfCells const &c2)
{
   return c1.fCost < c2.fCost;
}

/*******************************************************************************/
//Function Name: getPositnionInList
//Inputs:the cellID, the list
//Output: index of the cell in the list
//Description: it is used to search the index of a cell in a list
/*********************************************************************************/
list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID)
{
   for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
     if (it->index == cellID)
         return it;
   }
}


 /*******************************************************************************
 * Function Name: findFreeNeighborCell
  * Inputs: el índice de la celda
  * Output: a vector of free neighbor cells of the current cell
  * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
  * findFreeNeighborCell(i) es el conjunto de celdas del costmap tales que
  *             costmap[x,y] = FREE_SPACE, donde (x,y) son las coordenadas en el costmap del índice i
  *
*********************************************************************************/
vector <unsigned int> MyastarPlanner::findFreeNeighborCell (unsigned int CellID){

        unsigned int mx, my;
        costmap_->indexToCells(CellID,mx,my);
        vector <unsigned int>  freeNeighborCells;

        for (int x=-1;x<=1;x++)
          for (int y=-1; y<=1;y++){
            //check whether the index is valid
           if ((mx+x>=0)&&(mx+x < costmap_->getSizeInCellsX())&&(my+y >=0 )&&(my+y < costmap_->getSizeInCellsY())){
              //FREE_SPACE --> se trata de una constante perteneciente a la clase que esta indicada
              //Si se mira en la API de la clase, la documentacion en ROS, se pueden comprobar que hay
              //distintas constantes, para saber si hay o no hay obstaculo. En este caso FREE_SPACE, se supone que vale
              //cero según la documentacion pero devuelve la inversa, igual con el resto de constantes.
              if(costmap_->getCost(mx+x,my+y) == costmap_2d::FREE_SPACE   && (!(x==0 && y==0))){
                  unsigned int index = costmap_->getIndex(mx+x,my+y);
                  //ROS_INFO("Vecino encontrado!!");
                  freeNeighborCells.push_back(index);
              }
            }
        }
          return  freeNeighborCells;

      }


/*******************************************************************************/
//Function Name: isContains
//Inputs: the list, the cellID
//Output: true or false
//Description: it is used to check if a cell exists in the open list or in the closed list
/*********************************************************************************/
 bool isContains(list<coupleOfCells> & list1, int cellID)
 {
   for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
     if (it->index == cellID)
         return true;
  }
   return false;
}

double MyastarPlanner::getMoveCost(unsigned int here, unsigned int there) {
  //calculo el coste de moverme entre celdas adyacentes como la distancia euclídea.
  return calculateHCost(here,there);

}

/*******************************************************************************/
//Function Name: addNeighborCellsToOpenList
//Inputs: the open list, the neighbors Cells and the parent Cell
//Output:
//Description: it is used to add the neighbor Cells to the open list
/*********************************************************************************/
void MyastarPlanner::addNeighborCellsToOpenList(list<coupleOfCells> & OPL, vector <unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell) //,float tBreak)
{
        vector <coupleOfCells> neighborsCellsOrdered;
        unsigned int mx, my, theta = 0.0; //theta --> Proporción de reduccion de la seguridad
        double wx, wy;

        //ROS_INFO("Total vecinos: %d", neighborCells.size());
        for(uint i=0; i< neighborCells.size(); i++)
        {
          coupleOfCells CP;
          CP.index=neighborCells[i]; //insert the neighbor cell
          CP.parent=parent; //insert the parent cell

          //CALCULAR el gCost ----------------------------------------------------------------
          CP.gCost=gCostParent+1;//getMoveCost(parent,neighborCells[i]); --> esto solo calcula la distancia euclidea, que es una estimacion
          //para calcularlo, primero se ha de pasar el indice a celda
          costmap_->indexToCells(CP.index, mx, my);
          //Pasar de celda a coordenadas del mundo
          costmap_->mapToWorld(mx, my, wx, wy);
          //Acumular coste segun donde se encuentre la celda
          CP.gCost += footprintCost(wx, wy, theta);

          //calculate the hCost: Euclidian distance from the neighbor cell to the goalCell
          CP.hCost=calculateHCost(neighborCells[i],goalCell);
          //calculate fcost

          CP.fCost=CP.gCost+CP.hCost;
         // neighborsCellsOrdered.push_back(CP);
          OPL.push_back(CP);
        }
      }

//publicamos el plan para poder visualizarlo en rviz
void MyastarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
          if (!initialized_) {
              ROS_ERROR(
                      "This planner has not been initialized yet, but it is being used, please call initialize() before use");
              return;
          }

          //create a message for the plan
          nav_msgs::Path gui_path;
          gui_path.poses.resize(path.size());

          if (!path.empty()) {
              gui_path.header.frame_id = path[0].header.frame_id;
              gui_path.header.stamp = path[0].header.stamp;
          }

          // Extract the plan in world co-ordinates, we assume the path is all in the same frame
          for (unsigned int i = 0; i < path.size(); i++) {
              gui_path.poses[i] = path[i];
          }

          plan_pub_.publish(gui_path);
      }

}
