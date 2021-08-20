#include <TinyGPS++.h>
#include <ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
ros::NodeHandle  nh;

geometry_msgs::TwistWithCovarianceStamped gpsMsgVel;
sensor_msgs::NavSatFix gpsMsgLoc;
sensor_msgs::NavSatFix m8nRaw_1;
sensor_msgs::NavSatFix m8nRaw_2;
sensor_msgs::NavSatFix m8nRaw_3;

ros::Publisher gnssVel("multiGNSS_Vel", &gpsMsgVel);
ros::Publisher gnssLoc("multiGNSS_Loc", &gpsMsgLoc);
ros::Publisher gnss1("gnssRaw_1", &m8nRaw_1);
ros::Publisher gnss2("gnssRaw_2", &m8nRaw_2);
ros::Publisher gnss3("gnssRaw_3", &m8nRaw_3);

TinyGPSPlus gps1, gps2, gps3;
float lng_samples [3] = {0,0,0}; // ventana de datos de longitud
float lat_samples [3] = {0,0,0}; // ventana de datos de latitud
float alt_samples [3] = {0,0,0}; // ventana de datos de altitud
float covPos[9] = {0.01,0,0,0,0.01,0,0,0,0.01}; // matriz de covarianzas NavSatFix
float covVel[36] = {0.01,0,0,0,0,0 ,0,0.01,0,0,0,0 ,0,0,0.01,0,0,0 ,0,0,0,0.01,0,0 ,0,0,0,0,0.01,0 ,0,0,0,0,0,0.01 }; // matriz de covarianzas NavSatFix
float hdop_gps [3] = {0.0,0.0,0.0}; // hdop de cada gps en el instante n
float kmph_gps [3] = {0.0,0.0,0.0}; // velocidad en kmph de cada gps en el instante n
//float course_gps [3] = {0.0,0.0,0.0}; // direccion de cada gps en el instante n
float gps_sat [3] = {0.0,0.0,0.0}; // cantidad de satelites
float gain_gps[3] = {1.0,1.0,1.0}; //ganancias de los ublox
float valid_data [3]= {0,0,0}; // variables para validar los datos de posicion de gps
float valid_speed [3]= {0,0,0}; // variables para validar los datos de velocidad 
float lati  = 0, lati_ant = 0;// Latitud de salida
float longi = 0, lng_ant = 0; // longitud de salida 
float alt_ant = 0;
float speed_gps = 0, speed_ant = 0; // velocidad promedio
float twist_array[6] = {0,0,0,0,0,0}; //linear.x linear.y linear.z angular.x angular.y angular.z
byte flag_valid = 0;
float gnss_LNG = 0;
float gnss_LAT = 0;
float gnss_SPEED = 0 ;
float gnss_alt = 0; // Altitud de salida (este dato no es filtrado ya que no es necesario en nuestra aplicacion)
float course_mean = 0, course_mean_prev=0;
float lng_1 = 0, lng_2 = 0, lng_3 = 0; // calculos por cada longitud
float lat_1 = 0, lat_2 = 0, lat_3 = 0; // calculos por cada latitud
float speed_gnss[3]={ 0.0, 0.0, 0.0};
float sum_sat = 0;
float desv_lat[3]   = {0.005, 0.005, 0.005};
float desv_lng[3]   = {0.005, 0.005, 0.005};
float desv_speed[3] = {2.0,2.0,2.0};
float sigma_speed = 0.0;
unsigned int i=0; //contador
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);Serial2.begin(115200);Serial3.begin(115200);
  //Serial.println("Inicio: "); 
  nh.initNode();
  nh.advertise(gnssVel);
  nh.advertise(gnssLoc);
  nh.advertise(gnss1);
  nh.advertise(gnss2);
  nh.advertise(gnss3);
}

void loop()
{
  unsigned long ini = millis();
  data_gps(&Serial1, &gps1, 50,0);  
  data_gps(&Serial2, &gps2, 50,1);
  data_gps(&Serial3, &gps3, 50,2);
  
  sum_sat = ((valid_data[0]*gps_sat[0])+(valid_data[1]*gps_sat[1])+(valid_data[2]*gps_sat[2]))*1.0;
  
  lng_ant = longi;
  lati_ant = lati;
  speed_ant  = speed_gps;
  
  means();  
  
  flag_valid = (valid_data[0] && valid_data[1] && valid_data[2]);
  float sigma_lat = 0.0, sigma_lng = 0.0;
  
  //if (flag_valid){
  if (1){
    for (int j=0;j<3;j++)
    {
      sigma_lat = abs((lat_samples[j]-lati_ant)*100.0);
      sigma_lng = abs((lng_samples[j]-lng_ant)*100.0);
            
      if(sigma_lat > desv_lat[j] || sigma_lng > desv_lng[j]){
        valid_data[j]=0.0;
      }
      sigma_speed = abs((speed_gnss[j]-speed_ant));      
      if(sigma_speed > desv_speed[j]){
        valid_speed[j] = 0.0;
      }       
    }
    filtro();

    matrix_covariancePos(gnss_LAT,gnss_LNG,gnss_alt,lati_ant,lng_ant,alt_ant);
    matrix_covarianceVel(gnss_SPEED,0,0,0,0,0,speed_ant,0,0,0,0,0);
    
  }  

  //publish multiGNSS_data

  //Position
  gpsMsgLoc.header.stamp = nh.now();
  gpsMsgLoc.header.frame_id = "gps";
  gpsMsgLoc.latitude = gnss_LAT;
  gpsMsgLoc.longitude = gnss_LNG;
  gpsMsgLoc.altitude = alt_ant;
  for (int j=0;j<9;j++)
    gpsMsgLoc.position_covariance[j] = covPos[j];
  gnssLoc.publish(&gpsMsgLoc);
  //

  //Velocity
  gpsMsgVel.header.stamp = nh.now();
  gpsMsgVel.header.frame_id = "gps";
  gpsMsgVel.twist.twist.linear.x = cos(course_mean*PI/180.0)*gnss_SPEED;
  gpsMsgVel.twist.twist.linear.y = sin(course_mean*PI/180.0)*gnss_SPEED;
  gpsMsgVel.twist.twist.linear.z = 0.0;
  gpsMsgVel.twist.twist.angular.x = 0.0;
  gpsMsgVel.twist.twist.angular.y = 0.0;
  gpsMsgVel.twist.twist.angular.z = (course_mean - course_mean_prev)/(millis()-ini);
  for (int j=0;j<36;j++)
    gpsMsgVel.twist.covariance[j] = covVel[j];
  gnssVel.publish(&gpsMsgVel);
  
  //
  
  // publish raw_data1
  m8nRaw_1.header.stamp = nh.now();
  m8nRaw_1.header.frame_id = "gps";
  m8nRaw_1.latitude = lat_samples[0];
  m8nRaw_1.longitude = lng_samples[0];
  m8nRaw_1.altitude = alt_samples[0];
  gnss1.publish(&m8nRaw_1);
  //
  // publish raw_data2
  m8nRaw_2.header.stamp = nh.now();
  m8nRaw_2.header.frame_id = "gps";
  m8nRaw_2.latitude = lat_samples[1];
  m8nRaw_2.longitude = lng_samples[1];
  m8nRaw_2.altitude = alt_samples[1];
  gnss2.publish(&m8nRaw_2);
  //
  // publish raw_data3
  m8nRaw_3.header.stamp = nh.now();
  m8nRaw_3.header.frame_id = "gps";
  m8nRaw_3.latitude = lat_samples[2];
  m8nRaw_3.longitude = lng_samples[2];
  m8nRaw_3.altitude = alt_samples[2];
  gnss3.publish(&m8nRaw_3);
  //
  
  nh.spinOnce();
   
  lati_ant = gnss_LAT; 
  lng_ant  = gnss_LNG;
  alt_ant  = gnss_alt;  
  course_mean_prev =course_mean;
   
}

static void data_gps(HardwareSerial *port, TinyGPSPlus *gps,unsigned long ms, unsigned int i)
{
  unsigned long start = millis();
  
  do   
  {
    if (port->available()>0){
      while(port->available()>0)
        gps->encode(port->read());
      valid_data[i] = 1.0;
      valid_speed[i] = 1.0;
    }
     millis() - start < ms;
  } while (millis() - start < ms);   
  if (gps->location.isValid())
  {
    lat_samples [i] = gps->location.lat();
    lng_samples [i] = gps->location.lng();
    alt_samples [i] = gps->altitude.meters();
   }
  else
  {
    valid_data[i] = 0.0;
  }
  
  if (!gps->time.isValid())
  {
     valid_data[i] = 0;
  }
    
  if (gps->speed.isValid())
  {
     kmph_gps[i] = gps->speed.kmph();
   }  
  if (gps->hdop.isValid())
    {
      hdop_gps [i] = gps->hdop.hdop();
    }
    else{
      valid_data[i] = 0.0;
    }
  if (gps->satellites.isValid())
    {
      if(gps->satellites.value()<4)
        valid_data[i] = 0.0;
      gps_sat [i] = gps->satellites.value();
    }
    else{
      valid_data[i] = 0.0;
      }      
   unsigned long muestra = millis()-start;
}

void means(){
  // Longitud
  lng_1 =  valid_data[0]*gain_gps[0]*(gps_sat[0]*1.0)*lng_samples[0];
  lng_2 =  valid_data[1]*gain_gps[1]*(gps_sat[1]*1.0)*lng_samples[1];
  lng_3 =  valid_data[2]*gain_gps[2]*(gps_sat[2]*1.0)*lng_samples[2];
  
  longi = (lng_1+lng_2+lng_3)/sum_sat;
  
  // Latitud
  lat_1 =  valid_data[0]*gain_gps[0]*(gps_sat[0]*1.0)*lat_samples[0];
  lat_2 =  valid_data[1]*gain_gps[1]*(gps_sat[1]*1.0)*lat_samples[1];
  lat_3 =  valid_data[2]*gain_gps[2]*(gps_sat[2]*1.0)*lat_samples[2];
  lati  = (lat_1+lat_2+lat_3)/sum_sat;

  //Altitud

  gnss_alt = (alt_samples[0]+alt_samples[1]+alt_samples[2])/3.0; // promedio de altitud 
  
  //Velocidad
  speed_gnss[0] =  valid_data[0]*gain_gps[0]*kmph_gps[0];
  speed_gnss[1] =  valid_data[1]*gain_gps[1]*kmph_gps[1];
  speed_gnss[2] =  valid_data[2]*gain_gps[2]*kmph_gps[2];

  //course
  course_mean = valid_data[0]*(gps_sat[0]*1.0)*gps1.course.deg()+valid_data[0]*(gps_sat[0]*1.0)*gps2.course.deg()+valid_data[0]*(gps_sat[0]*1.0)*gps3.course.deg();
  course_mean = course_mean/sum_sat;
  
  speed_gps = (speed_gnss[0]+speed_gnss[1]+speed_gnss[2])/(valid_data[0]+valid_data[1]+valid_data[2]);
}

void  filtro(){

  if((valid_data[0]+valid_data[1]+valid_data[2])> 0 )
  {
    sum_sat = ((valid_data[0]*gps_sat[0])+(valid_data[1]*gps_sat[1])+(valid_data[2]*gps_sat[2]))*1.0;
    // Longitude
    
    lng_1 =  valid_data[0]*gain_gps[0]*lng_samples[0]*(gps_sat[0]*1.0);
    lng_2 =  valid_data[1]*gain_gps[1]*lng_samples[1]*(gps_sat[1]*1.0);
    lng_3 =  valid_data[2]*gain_gps[2]*lng_samples[2]*(gps_sat[2]*1.0);
    
    gnss_LNG = (lng_1+lng_2+lng_3)/(sum_sat);
    
    // Latitude
    lat_1 =  valid_data[0]*gain_gps[0]*lat_samples[0]*(gps_sat[0]*1.0);
    lat_2 =  valid_data[1]*gain_gps[1]*lat_samples[1]*(gps_sat[1]*1.0);
    lat_3 =  valid_data[2]*gain_gps[2]*lat_samples[2]*(gps_sat[2]*1.0);
    gnss_LAT  = (lat_1+lat_2+lat_3)/(sum_sat);
  
    //Velocity
    speed_gnss[0] =  valid_speed[0]*valid_data[0]*gain_gps[0]*kmph_gps[0];
    speed_gnss[1] =  valid_speed[1]*valid_data[1]*gain_gps[1]*kmph_gps[1];
    speed_gnss[2] =  valid_speed[2]*valid_data[2]*gain_gps[2]*kmph_gps[2];
    
  gnss_SPEED = (speed_gnss[0]+speed_gnss[1]+speed_gnss[2])/(valid_speed[0]*valid_data[0]+valid_speed[1]*valid_data[1]+valid_speed[2]*valid_data[2]);  
  }
}

void matrix_covariancePos(float x, float y, float z, float xp, float yp, float zp){
  covPos[0] = x-xp;
  covPos[1] = x-yp;
  covPos[2] = x-zp;
  covPos[3] = y-xp;
  covPos[4] = y-yp;
  covPos[5] = y-zp;
  covPos[6] = z-xp;
  covPos[7] = z-yp;
  covPos[8] = z-zp;
  }

void matrix_covarianceVel(float lx, float ly, float lz, float ax, float ay, float az, float lxp, float lyp, float lzp, float axp, float ayp, float azp){
  //linear.x linear.y linear.z angular.x angular.y angular.z // linear.x.previous linear.y.previous linear.z.previous angular.x.previous angular.y.previous angular.z.previous
  covVel[0]  = lx-lxp;
  covVel[1]  = lx-lyp;
  covVel[2]  = lx-lzp;
  covVel[3]  = lx-axp;
  covVel[4]  = lx-ayp;
  covVel[5]  = lx-azp;

  covVel[6]  = ly-lxp;
  covVel[7]  = ly-lyp;
  covVel[8]  = ly-lzp;
  covVel[9]  = ly-axp;
  covVel[10] = ly-ayp;
  covVel[11] = ly-azp;

  covVel[12] = lz-lxp;
  covVel[13] = lz-lyp;
  covVel[14] = lz-lzp;
  covVel[15] = lz-axp;
  covVel[16] = lz-ayp;
  covVel[17] = lz-azp;

  covVel[18] = ax-lxp;
  covVel[19] = ax-lyp;
  covVel[20] = ax-lzp;
  covVel[21] = ax-axp;
  covVel[22] = ax-ayp;
  covVel[23] = ax-azp;

  covVel[24] = ay-lxp;
  covVel[25] = ay-lyp;
  covVel[26] = ay-lzp;
  covVel[27] = ay-axp;
  covVel[28] = ay-ayp;
  covVel[29] = ay-azp;
  
  covVel[30] = az-lxp;
  covVel[31] = az-lyp;
  covVel[32] = az-lzp;
  covVel[33] = az-axp;
  covVel[34] = az-ayp;
  covVel[35] = az-azp;
  
  }
