/********************************************************
 *                                                      *
 *                                                      *
 *      campos_potenciales_sm.h		          	*
 *                                                      *
 *							*
 *		Dennis Mendoza				*
 *		01-04-2020                              *
 *                                                      *
 ********************************************************/
#define THRESHOLD_FOLLOWER 30
#define PI 3.1416

// State Machine 
int campos_potenciales_sm(float intensity, float *light_values, float *observations, int size, float laser_value, int  dest, int obs ,
	movement *movements  ,int *next_state ,float Mag_Advance ,float max_twist,float robot_x, float robot_y,
	float light_x, float light_y, float *darray, float* d_array,float *mm, float *mb, float m, float b, int *contador, int cont, int *pasos, int steps,
float *menor_x, float *menor_y, float m_x, float m_y,
	float *ini_x, float *ini_y, float inicio_x, float inicio_y, int *contador1, int cont1)
{

 int state = *next_state;
 int i;

 printf("intensity %f\n",intensity);
 printf("quantized destination %d\n",dest);
 printf("quantized obs %d\n",obs);
 printf("laser_value %f\n",laser_value);

 
int mayor_intensidad_luminosa = 0;
int sensor = 0;
float suma = 0;
float F_atr_r = 0;
float f_atr_r = 0;
float f_atr_theta = 0;
float epsilon_1 = 1;
float q_dest = 0;	
float d0 = 1.0; //5
float eta = 0.00001;
float suma_f_repulsiva_x = 0;
float suma_f_repulsiva_y = 0;	
float f_repulsiva_r = 0;
float f_repulsiva_theta = 0;
float f_total_r_x = 0;
float f_total_r_y = 0;
float f_total_r = 0;
float f_total_theta = 0;
float f_repulsiva_total_r = 0;
float f_repulsiva_total_theta = 0;

int n = 50; //20
for (int i=0;i<n-1;i++){
	darray[i]=d_array[i+1];	
}
darray[n-1]=sqrt((light_x-robot_x)*(light_x-robot_x)+(light_y-robot_y)*(light_y-robot_y));

/*printf("\ndarray: [%f",darray[0]);
for(int i=1;i<n-1;i++){
	printf(",%f",darray[i]);
}
printf("]\n");
*/				



 switch ( state ) {






        case 0:	
		//if ((darray[n-1]-darray[0])*(darray[n-1]-darray[0])<0.01*0.01){
		//	printf("\nSTUCK\n");
		//	*movements = generate_output(STOP,Mag_Advance,0.6*max_twist);
		//	*next_state=100; //100 para Bug 2 y 200 para Bug 1
		//}else{
		/*Encontramos la dirección de la luz*/		
/*
			    1	  0   	7
			    2	Robot  	6
			    3	  4	5
*/
		if(intensity > THRESHOLD_FOLLOWER){
			// hemos llegado 
			printf("Llegamos a la fuente luminosa\n");
			*movements=generate_output(STOP,Mag_Advance,0.6*max_twist);
	               				return 1;
		
		}else{
			for (int i=1;i<8;i++)		 	
			{
			    if( light_values[i] > light_values[sensor]){
					sensor = i; 
				}
	 		}
			//Creamos un vector que vaya hacia allá para definir la fuerza atractiva
			q_dest = intensity;
	 		F_atr_r = epsilon_1*sqrt(q_dest*q_dest);
			f_atr_r = 1;//F_atr_r/abs(F_atr_r);
	 		f_atr_theta = sensor*45*PI/180;
			printf("\nf_atr_r: %f",f_atr_r);
			printf("\nf_atr_theta: %f",f_atr_theta);


			suma = 0;					
			for(int i=0;i<size;i++){
				suma += observations[i];
				//printf("\nobservations[%i] = %f",i,observations[i]);				
			}
			    
			printf("\n\t\t\tsuma: %f, size*laser_value=%f\n",suma,size*laser_value);
			if((suma - size*laser_value)*(suma - size*laser_value)< 0.0001*0.0001){ 
			//Si no hay obstáculo en frente
				printf("No hay obstáculo\n");
				//Fuerza repulsiva = 0
				suma_f_repulsiva_x = 0;
				suma_f_repulsiva_y = 0;
				f_repulsiva_total_r = 0;
				f_repulsiva_total_theta = 0;
				
				*next_state = 0;

				f_total_r_x = f_atr_r*cos(f_atr_theta)+f_repulsiva_total_r*cos(f_repulsiva_total_theta);
				f_total_r_y = f_atr_r*sin(f_atr_theta)+f_repulsiva_total_r*sin(f_repulsiva_total_theta);
				f_total_r = 1; 
				f_total_theta = atan2(f_total_r_y,f_total_r_x);
				//printf("\nf_atr_r: %f",f_atr_r);						
				//printf("\nf_atr_theta: %f\t%f°\n\n",f_atr_theta,f_atr_theta*180/PI);
				//printf("\nf_repulsiva_r: %f",f_repulsiva_total_r);						
				//printf("\nf_repulsiva_theta: %f\t%f°\n\n",f_repulsiva_total_theta,f_repulsiva_total_theta*180/PI);
				//printf("\nf_total_r: %f",f_total_r);						
				//printf("\nf_total_theta: %f\t%f°\n\n",f_total_theta,f_total_theta*180/PI);						
		
				if(f_total_theta < PI/8 && f_total_theta > -PI/8){
					*movements = generate_output(FORWARD,Mag_Advance*f_total_r*0.3,f_total_theta*0.3);
					printf("\nFORWARD");
				}else if(f_total_theta >= PI/8 && f_total_theta <PI/2){
					*movements = generate_output(LEFT,Mag_Advance*f_total_r*0.3,f_total_theta*0.5);
					printf("\nLEFT");			
				}else if(f_total_theta <= -PI/8 && f_total_theta >-PI/2){
					*movements = generate_output(LEFT,Mag_Advance*f_total_r*0.3,f_total_theta*0.5);
					printf("\nRIGHT");			
				}else{
					*movements = generate_output(LEFT,Mag_Advance,f_total_theta*0.3);
					printf("\nBACKWARD---------------------------------------------------------");
				}


			}else{
			//Si hay obstáculo, entonces definimos la fuerza repulsiva a partir del punto medio y la distancia
				//Definimos la fuerza repulsiva
				//Revisamos si hay un obstáculo en algún lado para poder definir la fuerza, si no es 0
						
				//Si hay obstáculo(s) en frente, para cada uno calcular la fuerza repulsiva
				printf("Encontramos un obstáculo\n");
				//Consideramos cada lectura obstruida del láser como un obstáculo y calculamos su fuerza repulsiva
				suma_f_repulsiva_x = 0;
				suma_f_repulsiva_y = 0;		
				for(int i=0; i<size;i++){
					if(observations[i] < laser_value){
						//lectura obstruida, obstáculo
						f_repulsiva_r = eta*(1/observations[i]-1/d0)*(1/(observations[i]*observations[i]));
						f_repulsiva_theta = ((float)i/size)*PI-PI/2; //-PI/2 por el desfase, +PI por repulsivo

				//			printf("\nf_repulsiva_r: %f",f_repulsiva_r);
				//			printf("\ni: %i,(float)i/size: %f,(i/size)*PI: %f,f_repulsiva_theta: %f",i,(float)i/size,((float)i/size)*PI,f_repulsiva_theta);
						suma_f_repulsiva_x += f_repulsiva_r*cos(f_repulsiva_theta);
						suma_f_repulsiva_y += f_repulsiva_r*sin(f_repulsiva_theta);				
					}
				}
				f_repulsiva_total_r = -1;
				f_repulsiva_total_theta = atan2(suma_f_repulsiva_y,suma_f_repulsiva_x);
				//printf("\nf_repulsiva_total_r = %f\nf_repulsiva_total_theta = %f = %f°",f_repulsiva_total_r,f_repulsiva_total_theta,f_repulsiva_total_theta*180/PI);

				f_total_r_x = f_atr_r*cos(f_atr_theta)+f_repulsiva_total_r*cos(f_repulsiva_total_theta);
				f_total_r_y = f_atr_r*sin(f_atr_theta)+f_repulsiva_total_r*sin(f_repulsiva_total_theta);
				f_total_r = 1; 
				f_total_theta = atan2(f_total_r_y,f_total_r_x);
				//printf("\nf_atr_r: %f",f_atr_r);						
				//printf("\nf_atr_theta: %f\t%f°\n\n",f_atr_theta,f_atr_theta*180/PI);
				//printf("\nf_repulsiva_r: %f",f_repulsiva_total_r);						
				//printf("\nf_repulsiva_theta: %f\t%f°\n\n",f_repulsiva_total_theta,f_repulsiva_total_theta*180/PI);
				//printf("\nf_total_r: %f",f_total_r);						
				//printf("\nf_total_theta: %f\t%f°\n\n",f_total_theta,f_total_theta*180/PI);						
		
				if(f_total_theta < PI/8 && f_total_theta > -PI/8){
					*movements = generate_output(FORWARD,Mag_Advance*f_total_r*0.3,f_total_theta*0.3);
					printf("\nFORWARD");
				}else if(f_total_theta >= PI/8 && f_total_theta <PI/2){
					*movements = generate_output(LEFT,Mag_Advance*f_total_r*0.3,f_total_theta*0.5);
					printf("\nLEFT");			
				}else if(f_total_theta <= -PI/8 && f_total_theta >-PI/2){
					*movements = generate_output(LEFT,Mag_Advance*f_total_r*0.3,f_total_theta*0.5);
					printf("\nRIGHT");			
				}else{
					*movements = generate_output(LEFT,Mag_Advance*f_total_r*0.3,f_total_theta); //MagAdvance
					printf("\nBACKWARD---------------------------------------------------------");
				}
				
				*next_state = 1;
			}
			
			
			
			}
		//}
			
			break;	
	case 1:	 
				
		*movements = generate_output(FORWARD,Mag_Advance,0); //
		*next_state = 0;
		break;
/***************** RECUPERACIÓN CON BUG 2 **************/

	case 100:		
			// Estado inicial, orientarse hacia la fuente luminosa y caminar hacia ella
			// Girar hasta que el sensor 0 tenga el mayor valor de luminosidadfor(i = 1; i < 8; i++) 
			/*Calculamos los datos de la recta*/
			if(light_x-robot_x !=0){
				*mm=(light_y-robot_y)/(light_x-robot_x);
				*mb=robot_y-(*mm)*robot_x;
				printf("m=%f, b=%f\n",*mm, *mb);
				*movements = generate_output(STOP,Mag_Advance,max_twist);		
				*next_state = 101;
			}else{
				*movements=generate_output(STOP,Mag_Advance,0.6*max_twist);
	               				return 1;
			}	
			break;
	case 101:	
						
			// Estado inicial, orientarse hacia la fuente luminosa y caminar hacia ella
			// Girar hasta que el sensor 0 tenga el mayor valor de luminosidadfor(i = 1; i < 8; i++) 
		

			if(intensity > THRESHOLD_FOLLOWER){
				// hemos llegado 
				printf("Llegamos a la fuente luminosa\n");
				*movements=generate_output(STOP,Mag_Advance,0.6*max_twist);
	               				return 1;
		
			}else{
				for (int i=1;i<8;i++)		 	
				{
				    if( light_values[i] > light_values[sensor])
					sensor = i;
		 		}
		 		
		 		if(sensor > 4){
					// la luz se encuentra a la derecha
					sensor = -(8 - sensor);
					*movements=generate_output(RIGHT,Mag_Advance,0.6*max_twist);
		        		*next_state = 101;
		       		
				}else if(sensor == 0){
					// la luz se encuentra en frente
					
						suma = 0;					
						for(int i=0;i<size;i++){
							suma += observations[i];
						}
						printf("\n\t\t\tsuma: %f, size*laser_value=%f\n",suma,size*laser_value);
						if((suma - size*laser_value)*(suma - size*laser_value)< 0.0001*0.0001){ 
							//No hay obstáculo en frente
							printf("No hay obstáculo\n");
							*movements=generate_output(FORWARD,0.5*Mag_Advance,0.6*max_twist);
							*next_state = 101;
						}else{
							//Si hay obstáculo en frente, rodealo
							printf("Encontramos un obstáculo\n");
							*movements=generate_output(STOP,0*Mag_Advance,0*max_twist);
							*next_state = 2;
						}
				}else{ //la luz se encuentra a la izquierda
						*movements=generate_output(LEFT,Mag_Advance,0.6*max_twist);
						*next_state = 101;
		       		}
			}
			
			break;	
	
	case 2: 	// Alinearse perpendicular al obstáculo
			// Si cualquiera de los sensores de en frente o de a izquierda tienen algo, gira a la izquierda
		
			suma = 0;					
			for(int i=(int)(size/10);i<size;i++){
				suma += observations[i];
			}
			if(suma < size*laser_value*0.9){ 
				// hay obstáculo en frente
				printf("hay obstáculo, gira izquierda\n");
				*movements=generate_output(LEFT,0*Mag_Advance,0.6*max_twist);
				*next_state = 2;
			}else{
				//No hay obstáculo en frente, rodealo
				printf("No hay obstáculo\n");
				*movements=generate_output(STOP,0*Mag_Advance,0.6*max_twist);
				*next_state = 35;
			}
			break;
	case 35:	*movements = generate_output(FORWARD,Mag_Advance,max_twist);
			*next_state = 3; break;
	case 36:	*movements = generate_output(FORWARD,Mag_Advance,max_twist);
			*next_state = 37; break;
	case 37:	*movements = generate_output(FORWARD,Mag_Advance,max_twist);
			*next_state = 38; break;
	case 38:	*movements = generate_output(FORWARD,Mag_Advance,max_twist);
			*next_state = 3; break;			
	case 3:	
			//Empezamos a contar
			*pasos = steps + 1;
			//Estamos aproximadamente paralelos al obstáculo, con el obstáculo a la izquierda
			// Rodear el obstáculo
			//Guardar posición
			printf("\t\t\tcontador: %d\n",cont);
			printf("robot_x: %f\n", robot_x);
			printf("robot_y: %f\n", robot_y);
			printf("\t\t\t\tm=%f,b=%f\n",m,b);
		
			printf("\t\t\tdistancia(<0.03?)=%f\n",std::abs(-m*(robot_x)+1*(robot_y)-b)/(sqrt(m*m+1)));
		
			//(robot_x,robot_y) pertenece a la recta
			//robot_y == m*robot_x+b;
			//y=mx+b; A=m, B=1, C=b
		
			if(std::abs(-m*(robot_x)+1*(robot_y)-b)/(sqrt(m*m+1)) < 0.01){
				/*if(*contador != 0){
					printf("Vas bien, sigue adelante, contador=%d\n",*contador);
					*movements = generate_output(FORWARD,0.5*Mag_Advance,0.6*max_twist);
					*next_state = 3;
					*contador=cont+1;
				}else{*/
					printf("YA PASAMOS POR EL INICIO, contador=%d",*contador);
					*movements = generate_output(STOP,Mag_Advance,0.6*max_twist);
					*contador=0;
					cont=0;
					*next_state=0;//101 //Llegamos al punto mínimo /*MOD*/
				//}
			}else{
			
				if(intensity > THRESHOLD_FOLLOWER){
					// hemos llegado 
					printf("Llegamos a la fuente luminosa\n");
					*movements=generate_output(STOP,Mag_Advance,0.6*max_twist);
	               				return 1;
				}else{
					//Avanzamos hacia el frente, a menos que esté obstruido el 75% de la izquierda
					// No hemos llegado a la fuente luminosa
					// Girar hasta estar perpendicular al obstáculo
					suma = 0;					
					for(int i=(int)(size/10);i<size;i++){
						suma += observations[i];
					}
					if(suma < size*laser_value*0.9){ 
						// vas a chocar, gira a la izquierda
						printf("hay obstáculo, gira izquierda\n");
						*movements=generate_output(LEFT,Mag_Advance,0.6*max_twist);
						*next_state = 3;
					}else{
						//
						suma = 0;					
						for(int i=0;i<size;i++){
							suma += observations[i];
						}
						printf("\nsuma: %f, size*laser_value=%f",suma,size*laser_value);
					
						//Si ya no hay obstáculo a la derecha ni a la izquierda, gira a la derecha y avanza para acercarte al objeto
						if(suma >= size*laser_value){
							printf("Te estás saliendo, gira a la derecha y avanza");
							//Ya no hay objeto a nuestra izquierda, acercarse más
							*movements = generate_output(STOP,Mag_Advance,0.6*max_twist);
							*next_state = 4;
						}else{
							printf("Vas bien, sigue adelante\n");
							*movements = generate_output(FORWARD,0.5*Mag_Advance,0.6*max_twist);
							*next_state = 3;
					
						}
						//Avanza
						//No hay obstáculo en frente, rodealo
						//printf("No hay obstáculo\n");
						//*movements=generate_output(FORWARD,0.1*Mag_Advance,0*max_twist);
						//*next_state = 2;
					}
				}
				//*movements=generate_output(FORWARD,Mag_Advance,max_twist);
				//*next_state = 1;
			}
			/*if(*pasos > 150){
				*movements = generate_output(STOP,0.5*Mag_Advance,0.6*max_twist);
				*next_state = 0;
				pasos = 0;
			}*/
			break;
	case 4:		//Nos salimos, hay que regresar al objeto
			*movements = generate_output(RIGHT,Mag_Advance,0.6*max_twist);
			*next_state = 5;
			break;
	case 5: 	*movements = generate_output(FORWARD,0.5*Mag_Advance,0.6*max_twist);
			*next_state = 3;
			break;
/*********************************/
/******************* RECUPERACIÓN CON BUG 1 ************************/

	case 200:	// Estado inicial, orientarse hacia la fuente luminosa y caminar hacia ella
			// Girar hasta que el sensor 0 tenga el mayor valor de luminosidadfor(i = 1; i < 8; i++) 
		
			
			if(intensity > THRESHOLD_FOLLOWER){
				// hemos llegado 
				printf("Llegamos a la fuente luminosa\n");
				*movements=generate_output(STOP,Mag_Advance,0.6*max_twist);
	               				return 1;
		
			}else{
				for (int i=1;i<8;i++)		 	
				{
				    if( light_values[i] > light_values[sensor])
					sensor = i;
		 		}
		 		
		 		if(sensor > 4){
					// la luz se encuentra a la derecha
					sensor = -(8 - sensor);
					*movements=generate_output(RIGHT,Mag_Advance,0.6*max_twist);
		        		*next_state = 200;
		       		
				}else if(sensor == 0){
					// la luz se encuentra en frente
					
						suma = 0;					
						for(int i=0;i<size;i++){
							suma += observations[i];
						}
						printf("\n\t\t\tsuma: %f, size*laser_value=%f\n",suma,size*laser_value);
						if((suma - size*laser_value)*(suma - size*laser_value)< 0.0001*0.0001){ 
							//No hay obstáculo en frente
							printf("No hay obstáculo\n");
							*movements=generate_output(FORWARD,0.5*Mag_Advance,0.6*max_twist);
							*next_state = 200;
						}else{
							//Si hay obstáculo en frente, rodealo
							printf("Encontramos un obstáculo\n");
							*movements=generate_output(STOP,0*Mag_Advance,0*max_twist);
							*next_state = 201;
						}
				}else{ //la luz se encuentra a la izquierda
						*movements=generate_output(LEFT,Mag_Advance,0.6*max_twist);
						*next_state = 200;
		       		}
			}
			
			break;	
	case 201: 	// Alinearse perpendicular al obstáculo
			// Si cualquiera de los sensores de en frente o de a izquierda tienen algo, gira a la izquierda
			*menor_x = robot_x;	//Primer punto de contacto x
			*menor_y = robot_y;	//Primer punto de contacto y
			*ini_x = robot_x;
			*ini_y = robot_y;
			suma = 0;					
			for(int i=(int)(size/10);i<size;i++){
				suma += observations[i];
			}
			if(suma < size*laser_value*0.9){ 
				// hay obstáculo en frente
				printf("hay obstáculo, gira izquierda\n");
				*movements=generate_output(LEFT,0*Mag_Advance,0.6*max_twist);
				*next_state = 201;
			}else{
				//No hay obstáculo en frente, rodealo
				printf("No hay obstáculo\n");
				*movements=generate_output(STOP,0*Mag_Advance,0.6*max_twist);
				*next_state = 202;
			}
			break;
	
	case 202:	//Estamos aproximadamente paralelos al obstáculo, con el obstáculo a la izquierda
			// Rodear el obstáculo
			//Guardar posición
			printf("contador: %d\n",cont1);
			printf("ini_x: %f\n", *ini_x);
			printf("ini_y: %f\n", *ini_y);
			printf("robot_x: %f\n", robot_x);
			printf("robot_y: %f\n", robot_y);
			printf("menor_x: %f\n", *menor_x);
			printf("menor_y: %f\n", *menor_y);

			if( (*ini_x - robot_x)*(*ini_x - robot_x) < (0.04*0.04) && (*ini_y - robot_y)*(*ini_y - robot_y)<0.04*0.04){
				if(*contador1 < 3){
					printf("Vas bien, sigue adelante, contador=%d\n",*contador1);
					*movements = generate_output(FORWARD,0.5*Mag_Advance,0.6*max_twist);
					*next_state = 202;
					*contador1=cont+1;
				}else{
					printf("YA PASAMOS POR EL INICIO, contador=%d",*contador1);
					*movements = generate_output(STOP,Mag_Advance,0.6*max_twist);
					*next_state=0; //Llegamos al punto mínimo
				}
			}else{
				if((light_x-robot_x)*(light_x-robot_x)+(light_y-robot_y)*(light_y-robot_y)<(light_x-*menor_x)*(light_x-*menor_x)+(light_y-*menor_y)*(light_y-*menor_y)){
					*menor_x=robot_x;
					*menor_y=robot_y;
				}
				printf("\nmenor x: %f",*menor_x);
				printf("\nmenor_y: %f",*menor_y);
				if(intensity > THRESHOLD_FOLLOWER){
					// hemos llegado 
					printf("Llegamos a la fuente luminosa\n");
					*movements=generate_output(STOP,Mag_Advance,0.6*max_twist);
	               				return 1;
				}else{
					//Avanzamos hacia el frente, a menos que esté obstruido el 75% de la izquierda
					// No hemos llegado a la fuente luminosa
					// Girar hasta estar perpendicular al obstáculo
					suma = 0;					
					for(int i=(int)(size/10);i<size;i++){
						suma += observations[i];
					}
					if(suma < size*laser_value*0.9){ 
						// vas a chocar, gira a la izquierda
						printf("hay obstáculo, gira izquierda\n");
						*movements=generate_output(LEFT,Mag_Advance,0.6*max_twist);
						*next_state = 202;
					}else{
						//
						suma = 0;					
						for(int i=0;i<size;i++){
							suma += observations[i];
						}
						printf("\nsuma: %f, size*laser_value=%f",suma,size*laser_value);
					
						//Si ya no hay obstáculo a la derecha ni a la izquierda, gira a la derecha y avanza para acercarte al objeto
						if(suma >= size*laser_value){
							printf("Te estás saliendo, gira a la derecha y avanza");
							//Ya no hay objeto a nuestra izquierda, acercarse más
							*movements = generate_output(STOP,Mag_Advance,0.6*max_twist);
							*next_state = 203;
						}else{
							printf("Vas bien, sigue adelante\n");
							*movements = generate_output(FORWARD,0.5*Mag_Advance,0.6*max_twist);
							*next_state = 202;
					
						}
						//Avanza
						//No hay obstáculo en frente, rodealo
						//printf("No hay obstáculo\n");
						//*movements=generate_output(FORWARD,0.1*Mag_Advance,0*max_twist);
						//*next_state = 2;
					}
				}
				//*movements=generate_output(FORWARD,Mag_Advance,max_twist);
				//*next_state = 1;
			}
			break;
	case 203:	//Nos salimos, hay que regresar al objeto
			*movements = generate_output(RIGHT,Mag_Advance,0.6*max_twist);
			*next_state = 204;
			break;
	case 204: 	*movements = generate_output(FORWARD,0.5*Mag_Advance,0.6*max_twist);
			*next_state = 202;
			break;

	case 2012:	//Rodear hasta llegar al punto mínimo
			// Rodear el obstáculo
			//Guardar posición
			printf("YA LLEGAMOS");
			*contador1=0;
			cont1=0;
			printf("contador: %d\n",cont1);
			printf("ini_x: %f\n", *ini_x);
			printf("ini_y: %f\n", *ini_y);
			printf("robot_x: %f\n", robot_x);
			printf("robot_y: %f\n", robot_y);
			printf("menor_x: %f\n", *menor_x);
			printf("menor_y: %f\n", *menor_y);

			
				if((*menor_x-robot_x)*(*menor_x-robot_x) < 0.04*0.04 && (*menor_y-robot_y)*(*menor_y-robot_y) < 0.04*0.04 ){
					*movements=generate_output(STOP,Mag_Advance,max_twist);
					*next_state=200;					
				}else{
					if(intensity > THRESHOLD_FOLLOWER){
						// hemos llegado 
						printf("Llegamos a la fuente luminosa\n");
						*movements=generate_output(STOP,Mag_Advance,0.6*max_twist);
	               				return 1;
					}else{
						//Avanzamos hacia el frente, a menos que esté obstruido el 75% de la izquierda
						// No hemos llegado a la fuente luminosa
						// Girar hasta estar perpendicular al obstáculo
						suma = 0;					
						for(int i=(int)(size/10);i<size;i++){
							suma += observations[i];
						}
						if(suma < size*laser_value*0.9){ 
							// vas a chocar, gira a la izquierda
							printf("hay obstáculo, gira izquierda\n");
							*movements=generate_output(LEFT,Mag_Advance,0.6*max_twist);
							*next_state = 2012;
						}else{
							//
							suma = 0;					
							for(int i=0;i<size;i++){
								suma += observations[i];
							}
							printf("\nsuma: %f, size*laser_value=%f",suma,size*laser_value);
					
							//Si ya no hay obstáculo a la derecha ni a la izquierda, gira a la derecha y avanza para acercarte al objeto
							if(suma >= size*laser_value){
								printf("Te estás saliendo, gira a la derecha y avanza");
								//Ya no hay objeto a nuestra izquierda, acercarse más
								*movements = generate_output(STOP,Mag_Advance,0.6*max_twist);
								*next_state = 2013;
							}else{
								printf("Vas bien, sigue adelante\n");
								*movements = generate_output(FORWARD,0.5*Mag_Advance,0.6*max_twist);
								*next_state = 2012;
					
							}
							//Avanza
							//No hay obstáculo en frente, rodealo
							//printf("No hay obstáculo\n");
							//*movements=generate_output(FORWARD,0.1*Mag_Advance,0*max_twist);
							//*next_state = 2;
						}
					}
					//*movements=generate_output(FORWARD,Mag_Advance,max_twist);
					//*next_state = 1;
				}
			
			break;
	case 2013:		//Nos salimos, hay que regresar al objeto
			*movements = generate_output(RIGHT,Mag_Advance,0.6*max_twist);
			*next_state = 2014;
			break;
	case 2014: 	*movements = generate_output(FORWARD,0.5*Mag_Advance,0.6*max_twist);
			*next_state = 2012;
			break;
/*******************************************/

        case 99:
	               *movements=generate_output(STOP,Mag_Advance,0.6*max_twist);
	               *next_state = 99;
			return 1;
	                break;
	
	
	default:
			//printf("State %d not defined used ", state);
	                *movements=generate_output(STOP,Mag_Advance,0.6*max_twist);
	                *next_state = 99;
	                break;
	
 	               
 	}

 
 printf("Next State: %d\n", *next_state);
 return 0;
}



                 
