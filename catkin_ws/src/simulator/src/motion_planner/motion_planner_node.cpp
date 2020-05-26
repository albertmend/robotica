/***********************************************
*                                              *
*      motion_planner_node.cpp                 *
*                                              *
*      Jesus Savage                            *
*      Diego Cordero                           *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 17-2-2020                 *
*                                              *
*                                              *
************************************************/

#include "ros/ros.h"
#include "math.h"
#include "../utilities/simulator_structures.h"
#include "../utilities/random.h"
#include "motion_planner_utilities.h"
#include "../state_machines/light_follower.h"
#include "../state_machines/sm_avoidance.h"
#include "../state_machines/sm_avoidance_destination.h"
#include "../state_machines/sm_destination.h"
#include "../state_machines/campos_potenciales_sm.h"
#include "../state_machines/user_sm.h"
#include "../state_machines/dijkstra.h"
#include "../state_machines/dfs.h"
#include "clips_ros/SimuladorRepresentation.h"
#include "../behaviors/oracle.h"
#include "../action_planner/action_planner.h"
#include "../action_planner/pf_action_planner.h"



int main(int argc ,char **argv)
{
    ros::init(argc ,argv ,"simulator_motion_planner_node");
    ros::NodeHandle n;
    ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub",0, parametersCallback);
    ros::Subscriber sub = n.subscribe("/scan", 10, laserCallback);
    SimuladorRepresentation::setNodeHandle(&n);
    ros::Rate r(20);


    float lidar_readings[512];
    float light_readings[8];
    
    int i;
    int tmp;
    int sensor;
    int est_sig;
    int q_light;
    int q_inputs;
    int flagOnce;
    int flg_finish;
    int mini_sm=1;
    int cta_steps;
    int flg_result;
    int flg_noise=0;
    
    float result;
    float final_x;
    float final_y;
    float intensity;
    float max_advance;
    float max_turn_angle;
    float noise_advance;
    float noise_angle;
    float angle;
    int door[2];
    float meta_x;
    float meta_y;
    
/****/
  float menor_x;
    float menor_y;
    float m_x=0;
    float m_y=0;
    float x_inicial=0;
    float y_inicial=0;
    float x_ini=0;
    float y_ini=0;
    int contador=0;
    int cont=0;
    int contador1=0;
    int cont1=0;
    float mm=0;
    float m=0;
    float mb=0;
    float b=0;    
    int pasos = 0;
    int stepss = 0;

    float d0[50];
    float d_0[50];

/****/

    char path[100];
    char object_name[20];


    movement movements;
    step steps[200];
    step graph_steps[200];

    // it sets the environment's path
    strcpy(path,"./src/simulator/src/data/");

    while( ros::ok()  )
    {
        flagOnce = 1;
        cta_steps = 0;
        mini_sm =1;

        while( params.run )
        {
            // it gets sensory data
            ros::spinOnce();

            if (!params.turtle)
            {
                get_light_values(&intensity,light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h

                get_lidar_values(lidar_readings,params.robot_x,
                                 params.robot_y,params.robot_theta,params.turtle); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            }
            else
            {
                get_light_values_turtle(&intensity,light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
                for( i = 0; i < 512; i++)
                    lidar_readings[i] = lasers[i];
            }

            // it quantizes the sensory data
            q_light = quantize_light(light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            
            if(params.noise )
                q_inputs = quantize_laser_noise(lidar_readings,params.laser_num_sensors,params.laser_value); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            else
                q_inputs = quantize_laser(lidar_readings,params.laser_num_sensors,params.laser_value); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h


            max_advance = params.robot_max_advance;
            max_turn_angle = params.robot_turn_angle;

            switch ( params.behavior)
            {

            case 1:
                // This function sends light sensory data to a function that follows a light source and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/light_follower.h
                flg_result = light_follower(intensity, light_readings,&movements,max_advance,max_turn_angle);
                if(flg_result == 1) stop();
                break;

            case 2:
                // This function sends light sensory data to an state machine that follows a light source and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_destination.h
                if(flagOnce)
                {
                    est_sig = 1;
                    flagOnce = 0;
                }
                flg_result = sm_destination(intensity,q_light,&movements,&est_sig,params.robot_max_advance,params.robot_turn_angle);
                if(flg_result == 1) stop();

                break;

            case 3:
                // This function sends quantized sensory data to an state machine that avoids obstacles and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_avoidance.h
                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                sm_avoid_obstacles(q_inputs,&movements,&est_sig ,params.robot_max_advance ,params.robot_turn_angle);
                break;

            case 4:
                // This function sends quantized sensory data to an state machine that follows a light source and avoids obstacles
                // and it issues the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_avoidance_destination.h
                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,
                                                    params.robot_max_advance ,params.robot_turn_angle);

                if(flg_result == 1) stop();
                break;

            case 5:
                // This function sends the intensity and the quantized sensory data to a Clips node and it receives the actions that
                // the robot needs to perfom to avoid obstacles and reach a light source according to first order logic
                // It is located in ~/catkin_ws/src/simulator/src/behaviors/oracle.h
                result=oracle_clips(intensity,q_light,q_inputs,&movements,max_advance ,max_turn_angle);
                if(result == 1.0) stop();
                break;


            case 6:
                // it finds a path from the origen to a destination using depth first search
                if(flagOnce)
                {
                    for(i = 0; i < 200; i++) steps[i].node = -1;

                    // it finds a path from the origen to a destination using first search
                    dfs(params.robot_x ,params.robot_y ,params.light_x ,params.light_y ,params.world_name,steps);
                    print_algorithm_graph (steps);
                    i = 0;
                    final_x = params.light_x;
                    final_y = params.light_y;
                    set_light_position(steps[i].x,steps[i].y);
                    printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                    flagOnce = 0;
                    flg_finish=0;
                    est_sig = 0;
                    movements.twist=0.0;
                    movements.advance =0.0;
                }
                else
                {
                    //flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,                                                        //params.robot_max_advance ,params.robot_turn_angle);
                    flg_result = oracle_clips(intensity,q_light,q_inputs,&movements,max_advance ,max_turn_angle);

                    if(flg_result == 1)
                    {
                        if(flg_finish == 1)
                            stop();
                        else
                        {
                            if(steps[i].node != -1)
                            {
                                set_light_position(steps[i].x,steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                                printf("Node %d\n",steps[i].node);
                                i++;
                                //printf("type a number \n");
                                //scanf("%d",&tmp);
                            }
                            else
                            {
                                set_light_position(final_x,final_y);
                                printf("New Light %d: x = %f  y = %f \n",i,final_x,final_y);
                                flg_finish = 1;
                            }
                        }
                    }
                }

                break;

            case 7:
                if(flagOnce)
                {
                    for(i = 0; i < 200; i++)steps[i].node=-1;
                    // it finds a path from the origen to a destination using the Dijkstra algorithm
                    door[1]=-1;
		    door[0]=-1;
dijkstra(params.robot_x ,params.robot_y ,params.light_x ,params.light_y ,params.world_name,steps,door);
                    print_algorithm_graph (steps);
                    i=0;
                    final_x=params.light_x;
                    final_y= params.light_y;
                    set_light_position(steps[i].x,steps[i].y);
                    printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                    flagOnce = 0;
                    flg_finish=0;
                    est_sig = 0;
                    movements.twist=0.0;
                    movements.advance =0.0;
                }
                else
                {
                    flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,
                                                        params.robot_max_advance ,params.robot_turn_angle);

                    if(flg_result == 1)
                    {
                        if(flg_finish == 1) stop();
                        else
                        {
                            if(steps[i].node != -1)
                            {
                                set_light_position(steps[i].x,steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                                printf("Node %d\n",steps[i].node);
                                i++;
                                //printf("type a number \n");
                                //scanf("%d",&tmp);
                            }
                            else
                            {
                                set_light_position(final_x,final_y);
                                printf("New Light %d: x = %f  y = %f \n",i,final_x,final_y);
                                flg_finish=1;
                            }
                        }
                    }
                }
                break;

            case 8:
                // Here it goes your code for selection 8
                if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                user_sm(intensity,light_readings, lidar_readings, params.laser_num_sensors,params.laser_value,
                        q_light,q_inputs,&movements,&est_sig ,params.robot_max_advance ,params.robot_turn_angle);
                break;

            case 9:

                flg_result=light_follower(intensity, light_readings,&movements,max_advance,max_turn_angle);
                if(flg_result == 1)
                    set_light_position(.5,.25);

                break;

            
            case 10:

		action_planner(params.robot_x, params.robot_y,params.robot_theta,&movements);

                break;

	    case 11:
		if(flagOnce)
                {
                    est_sig = 3000;
                    flagOnce = 0;
                }
		if(est_sig==3000){
			//Primero se crea el plan para cada goto, se llama a dijkstra y obtenemos steps
			printf("\nestado: 3000\n");
			pf_action_planner(params.robot_x,params.robot_y,params.robot_theta,&movements,params.world_name,steps,&est_sig);
			printf("\nEl action planer ha devuelto el plan\n");
		}
		else if(est_sig==3001){
		    //los steps obtenidos de dijkstra se convierten en el light position uno por uno
		    printf("\nestado: 3001\n");
		    i=0;
		    set_light_position(steps[i].x,steps[i].y);
		    printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                    flg_finish=0;
                    est_sig = 0;
                    movements.twist=0.0;
                    movements.advance =0.0;
                }
		else if (est_sig==3020){
			printf("\nestado: 3020\n");
		    	movements.twist=0.0;
                	movements.advance =max_advance*0.2;
			printf("\nsteps[%d].x: %f,steps[%d].y: %f,params.robot_x:%f,params.robot_y:%f\n",i,steps[i-1].x,i,steps[i-1].y,params.robot_x,params.robot_y);
			printf("\ndistancia a la luz: %f, max_advance: %f\n",sqrt(pow(steps[i-1].y-params.robot_y,2)+pow(steps[i-1].x-params.robot_x,2)),max_advance);
			printf("\nq_inputs: %d\n",q_inputs);
			//si la posicion actual del robot está suficientemente cerca, muevete hacia el punto
			if (sqrt(pow(steps[i-1].y-params.robot_y,2)+pow(steps[i-1].x-params.robot_x,2))<0.05){
			printf("\nHemos llegado al otro lado de la puerta\n");
			est_sig=0;}
			else if (q_inputs>0){
				printf("\nParece que hay un obstáculo. Nodos de la ruta vieja:\n");
				for(int i=0;i<20;i++){
					printf("\nsteps[%d].node=%d\n",i,steps[i].node);
  		    		}
				//si el punto no está cerca, 
				int k=0;
				while(steps[k].node!=-1){
				k++;
				}
				door[1]=steps[i-1].node;
			    	door[0]=steps[i-2].node;
				printf("nodo = %d,meta_x=steps[%d].x=%f,meta_y=steps[%d].y=%f",steps[k-1].node,k-1,steps[k-1].x,k-1,steps[k-1].y);
				meta_x=steps[k-1].x;
				meta_y=steps[k-1].y;
				for(i = 0; i < 200; i++)steps[i].node=-1;
                		dijkstra(params.robot_x,params.robot_y ,meta_x ,meta_y ,params.world_name,steps,door);
				printf("\nNodos de la ruta nueva\n");
				for(int i=0;i<20;i++){
					printf("\nsteps[%d].node=%d\n",i,steps[i].node);
  		    		}
				print_algorithm_graph (steps);
				i=0;
				set_light_position(steps[i].x,steps[i].y);
                	        printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
				est_sig=0;
			}else{
				printf("\nNo está entrando a ningún if\n");
			}
		}
                else
                {
			//se manda a llamar el algoritmo de evasión de obstáculos
			printf("\nLlamando al algoritmo de evasión de obstáculos\n");
                    //flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,
                    //                                  params.robot_max_advance ,params.robot_turn_angle);

/*********************/
			flg_result=campos_potenciales_sm(intensity,light_readings, lidar_readings, params.laser_num_sensors,params.laser_value, q_light, q_inputs, &movements,&est_sig ,params.robot_max_advance ,params.robot_turn_angle,params.robot_x ,params.robot_y,params.light_x,params.light_y,d0,d_0,&mm,&mb, m, b,&contador1,cont1, &pasos, stepss,&menor_x,&menor_y, m_x, m_y,&x_ini,&y_ini, x_inicial, y_inicial, &contador,cont);
			m_x=menor_x;
			m_y=menor_y;
			x_inicial=x_ini;
			y_inicial=y_ini;
			cont=contador;
			for(int i=0;i<50;i++){
				d_0[i]=d0[i];
			}	
			
			m=mm;
			b=mb;
			cont1=contador1;
			stepss = pasos;
/*********************/

                    if(flg_result == 1) // Si llegó a la fuente luminosa
                    {
			/*INSERTED*/
			
			cont1=0; contador1=0; cont=0;contador=0;
			/*INSERTED*/
                        if(flg_finish == 1) {
				//stop();
				est_sig=3000;	//regresa a action_planer
			}
                        else
                        {
                            if(steps[i].node != -1)
                            {
				//Si llegamos a la puerta del nodo 6 al 10
				if(
((steps[i-1].node==6  && steps[i].node==10)  || (steps[i-1].node==10 && steps[i].node==6 )) ||
((steps[i-1].node==3  && steps[i].node==2 )  || (steps[i-1].node==2  && steps[i].node==3 )) ||
((steps[i-1].node==1  && steps[i].node==13)  || (steps[i-1].node==13 && steps[i].node==1 )) ||
((steps[i-1].node==14 && steps[i].node==20)  || (steps[i-1].node==20 && steps[i].node==14)) ||
((steps[i-1].node==12 && steps[i].node==7 )  || (steps[i-1].node==7  && steps[i].node==12)) ||
((steps[i-1].node==7  && steps[i].node==8 )  || (steps[i-1].node==8  && steps[i].node==7 )) ||
((steps[i-1].node==11 && steps[i].node==23)  || (steps[i-1].node==23 && steps[i].node==11)) ||
((steps[i-1].node==17 && steps[i].node==16)  || (steps[i-1].node==16 && steps[i].node==17)) ||
((steps[i-1].node==16 && steps[i].node==21)  || (steps[i-1].node==21 && steps[i].node==16))

){
					angle=atan2(steps[i].y-steps[i-1].y,steps[i].x-steps[i-1].x);
					movements.twist=angle-params.robot_theta;
                    			movements.advance =0.0;
					est_sig=3020;

			        }

                                set_light_position(steps[i].x,steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n",i,steps[i].x,steps[i].y);
                                printf("Node %d\n",steps[i].node);
                                i++;
                                //printf("type a number \n");
                                //scanf("%d",&tmp);
                            }
                            else
                            {
                                flg_finish=1;
				
                            }
                        }
                    }
                }

                break;


	     case 12:
			if(flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                flg_result=campos_potenciales_sm(intensity,light_readings, lidar_readings, params.laser_num_sensors,params.laser_value, q_light, q_inputs, &movements,&est_sig ,params.robot_max_advance ,params.robot_turn_angle,params.robot_x ,params.robot_y,params.light_x,params.light_y,d0,d_0,&mm,&mb, m, b,&contador1,cont1, &pasos, stepss,&menor_x,&menor_y, m_x, m_y,&x_ini,&y_ini, x_inicial, y_inicial, &contador,cont);
		
		//d_1=d1;
		//d_2=d2;
		//d_3=d3;
		//d_4=d4;
		//d_5=d5;
		m_x=menor_x;
		m_y=menor_y;
		x_inicial=x_ini;
		y_inicial=y_ini;
		cont=contador;
		for(int i=0;i<20;i++){
			d_0[i]=d0[i];
		}	
		
		m=mm;
		b=mb;
		cont1=contador1;
		stepss = pasos;
		if(flg_result == 1) {stop(); cont1=0; contador1=0; cont=0;contador=0;}
			break;
             default:
                    printf(" ******* SELECTION NO DEFINED *******\n");
                    movements.twist = 3.1416/4;
                    movements.advance = .03;
                break;
            }

            ros::spinOnce();
            printf("\n\n             MOTION PLANNER \n________________________________\n");
            printf("Light: x = %f  y = %f \n",params.light_x,params.light_y);
            printf("Robot: x = %f  y = %f \n",params.robot_x,params.robot_y);
            printf("Step: %d \n",cta_steps++);
            printf("Movement: twist: %f advance: %f \n" ,movements.twist ,movements.advance );

            flg_noise = params.noise;

            move_robot(movements.twist,movements.advance,lidar_readings);
            ros::spinOnce();
            new_simulation = 0;
            r.sleep();
        }
        ros::spinOnce();
        r.sleep();
    }
}
