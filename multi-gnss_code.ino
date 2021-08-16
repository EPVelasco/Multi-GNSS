#include <TinyGPS++.h>
#include <ros.h>
#include <sensor_msgs/NavSatFix.h>

ros::NodeHandle  nh;
sensor_msgs::NavSatFix gpsMsg;
ros::Publisher gps("multiGNSS", &gpsMsg);

TinyGPSPlus gps1, gps2, gps3;
float lng_samples [3] = {0,0,0}; // ventana de datos de longitud
float lat_samples [3] = {0,0,0}; // ventana de datos de latitud
float hdop_gps [3] = {0.0,0.0,0.0}; // hdop de cada gps en el instante n
float kmph_gps [3] = {0.0,0.0,0.0}; // velocidad en kmph de cada gps en el instante n
//float course_gps [3] = {0.0,0.0,0.0}; // direccion de cada gps en el instante n
float gps_sat [3] = {0.0,0.0,0.0}; // cantidad de satelites
float gain_gps[3] = {1.0,1.0,1.0}; //ganancias de los ublox
float valid_data [3]= {0,0,0}; // variables para validar los datos de posicion de gps
float valid_speed [3]= {0,0,0}; // variables para validar los datos de velocidad 
float lati  = 0, lati_ant = 0;// Latitud de salida
float longi = 0, lng_ant = 0; // longitud de salida 
float speed_gps = 0, speed_ant = 0; // velocidad promedio
byte flag_valid = 0;
float gnss_LNG = 0;
float gnss_LAT = 0;
float gnss_SPEED = 0 ;

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
  nh.advertise(gps);
}

void loop()
{
  unsigned long ini = millis();
  data_gps(&Serial1, &gps1, 100,0);  
  data_gps(&Serial2, &gps2, 100,1);
  data_gps(&Serial3, &gps3, 100,2);
 
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
      /*Serial.print("Sla:");
      Serial.print(sigma_lat,8);
      Serial.print(" Sln:");
      Serial.print(sigma_lng,8);
      Serial.print(" ");*/ 
      sigma_speed = abs((speed_gnss[j]-speed_ant));      
      if(sigma_speed > desv_speed[j]){
        valid_speed[j] = 0.0;

      }     
  
    }
    filtro();
    
  }  

  //publish GPS_data

  gpsMsg.header.stamp = nh.now();
  gpsMsg.header.frame_id = "map";
  gpsMsg.latitude = gnss_LAT;
  gpsMsg.longitude = gnss_LNG;
  //gpsMsg.altitude = 0.0;
  gps.publish(&gpsMsg);
  nh.spinOnce();
   
  /*.print("LAT: ");
  Serial.print(gnss_LAT,8);
  Serial.print("  LONG: ");
  Serial.print(gnss_LNG,8); 
  Serial.print("  SPEED: ");
  Serial.print(gnss_SPEED,5);
  Serial.print("  time_ms: ");
  Serial.println(millis()-ini);*/
    
  lati_ant = gnss_LAT; 
  lng_ant = gnss_LNG;
    
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
  /*Serial.print(valid_data[i]);
  Serial.print("GPS");
  Serial.print(i);
  Serial.print(": ");      
  Serial.print(F("Location: "));*/ 
  
   
  if (gps->location.isValid())
  {
    lat_samples [i] = gps->location.lat();
    lng_samples [i] = gps->location.lng();
    
    /*Serial.print(gps->location.lat(), 8);
    Serial.print(F(","));
    Serial.print(gps->location.lng(), 8);*/
   }
  else
  {
    valid_data[i] = 0.0;
    //Serial.print(F("INVALID1"));
  }
  
  //Serial.print(F("  Date/Time: "));

  
  if (!gps->time.isValid())
  {
    //Serial.print(F("INVALID2   "));
     valid_data[i] = 0;
  }
  /*else
  {
    if (gps->time.hour() < 10) Serial.print(F("0"));
      Serial.print(gps->time.hour());
      Serial.print(F(":"));
      if (gps->time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps->time.minute());
      Serial.print(F(":"));
      if (gps->time.second() < 10) Serial.print(F("0"));
        Serial.print(gps->time.second());
      Serial.print(F("."));
      if (gps->time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps->time.centisecond());
      Serial.print("  ");
  }*/
  
  if (gps->speed.isValid())
  {
     kmph_gps[i] = gps->speed.kmph();
    // Serial.print(kmph_gps[i]);
  }
  /*else
      Serial.print(F("INVALID3   "));

   Serial.print("  Validaciones: ");
   Serial.print(valid_data[i]);
   Serial.print("  ");
   Serial.print("Prom: ");
   Serial.print(lati,8);
   Serial.print("  ");
   Serial.print(longi,8); 
   Serial.print("  ");
   Serial.print(speed_gps,5);
   Serial.print("  ");
   Serial.print("Filtro: ");
   Serial.print(gnss_LAT,8);
   Serial.print("  ");
   Serial.print(gnss_LNG,8); 
   Serial.print("  ");
   Serial.print(gnss_SPEED,5);
   Serial.print(" hd:");*/
  
  if (gps->hdop.isValid())
    {
      hdop_gps [i] = gps->hdop.hdop();
      //Serial.print(hdop_gps [i]);
      //Serial.print(" sat:");
    }
    else{
      valid_data[i] = 0.0;
      //Serial.print(F("INVALID4   "));
    }
  if (gps->satellites.isValid())
    {
      if(gps->satellites.value()<4)
        valid_data[i] = 0.0;
      gps_sat [i] = gps->satellites.value();
      //Serial.print(gps_sat [i]); 
      //Serial.print("  ");     
    }
    else{
      valid_data[i] = 0.0;
      //Serial.print(F("INVALID5   "));
      }
      //Serial.println();  
      
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

  //Velocidad
  speed_gnss[0] =  valid_data[0]*gain_gps[0]*kmph_gps[0];
  speed_gnss[1] =  valid_data[1]*gain_gps[1]*kmph_gps[1];
  speed_gnss[2] =  valid_data[2]*gain_gps[2]*kmph_gps[2];
  
  speed_gps = (speed_gnss[0]+speed_gnss[1]+speed_gnss[2])/(valid_data[0]+valid_data[1]+valid_data[2]);
}

void  filtro(){

  if((valid_data[0]+valid_data[1]+valid_data[2])> 0 )
  {
    sum_sat = ((valid_data[0]*gps_sat[0])+(valid_data[1]*gps_sat[1])+(valid_data[2]*gps_sat[2]))*1.0;
    // Longitud
    
    lng_1 =  valid_data[0]*gain_gps[0]*lng_samples[0]*(gps_sat[0]*1.0);
    lng_2 =  valid_data[1]*gain_gps[1]*lng_samples[1]*(gps_sat[1]*1.0);
    lng_3 =  valid_data[2]*gain_gps[2]*lng_samples[2]*(gps_sat[2]*1.0);
    
    gnss_LNG = (lng_1+lng_2+lng_3)/(sum_sat);
    
    // Latitud
    lat_1 =  valid_data[0]*gain_gps[0]*lat_samples[0]*(gps_sat[0]*1.0);
    lat_2 =  valid_data[1]*gain_gps[1]*lat_samples[1]*(gps_sat[1]*1.0);
    lat_3 =  valid_data[2]*gain_gps[2]*lat_samples[2]*(gps_sat[2]*1.0);
    gnss_LAT  = (lat_1+lat_2+lat_3)/(sum_sat);
  
    //Velocidad
    speed_gnss[0] =  valid_speed[0]*valid_data[0]*gain_gps[0]*kmph_gps[0];
    speed_gnss[1] =  valid_speed[1]*valid_data[1]*gain_gps[1]*kmph_gps[1];
    speed_gnss[2] =  valid_speed[2]*valid_data[2]*gain_gps[2]*kmph_gps[2];
    /*Serial.print("  Validaciones: ");
    Serial.print(valid_data[0]);
    Serial.print(" ");
    Serial.print(valid_data[1]);
    Serial.print(" ");
    Serial.println(valid_data[2]);*/
    
  gnss_SPEED = (speed_gnss[0]+speed_gnss[1]+speed_gnss[2])/(valid_speed[0]*valid_data[0]+valid_speed[1]*valid_data[1]+valid_speed[2]*valid_data[2]);  
  }
}
