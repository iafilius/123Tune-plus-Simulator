// Conversion functions


// Turns bluetooth binary into decimal RPM
uint32_t TuneRPM2decimal(uint8_t MSB, uint8_t LSB)
{
  uint32_t MSB_dec;
  uint32_t LSB_dec;
  uint32_t value;
  double MSB_weight=800/1;  // From readout digital advance curve Scaling MSB != scaling LSB
  double LSB_Weight=50/1;   // From roudaout advance curve, is not exact due to jump in readout table.
  char str[2];      // char string for hex number

  str[0]=MSB;     // fill string, to convert a single hex char
  str[1]='\0';
  MSB_dec=strtol(str,NULL,16);

  str[0]=LSB;                     // fill string, to convert a single hex char
  str[1]='\0';
  LSB_dec=strtol(str,NULL,16);

  value=MSB_dec*MSB_weight + LSB_dec*LSB_Weight;
return(value);
}


// Turns bluetooth binary into decimal position No 1 -- 10
// even numbers are RPM Curve positions times 2 
// uneven numbers are Degrees Curve position times 2 +1
// Curve No 1: RPM   -> 2 (*2) // First RPM  fixated on 500, so 2 is illegal
// Curve No 1: Degrees  -> 3
// Curve No 2: RPM  -> 4 (*2)
// Curve No 2: Degrees  -> 5 (*2+1)
// Curve No 3: RPM      -> 6 (*2)
// Curve No 3: Degrees  -> 7 (*2+1)
// Curve No 4: RPM  -> 8 (*2)
// Curve No 4: Degrees  -> 9 (*2+1)
// Curve No 5: RPM      -> 10 (*2)
// Curve No 5: Degrees  -> 11 (*2+1)
// Curve No 6: RPM  -> 12 (*2)
// Curve No 6: Degrees  -> 13 (*2+1)
// Curve No 7: RPM      -> 14 (*2)
// Curve No 7: Degrees  -> 15 (*2+1)
// Curve No 8: RPM  -> 16 (*2)
// Curve No 8: Degrees  -> 17 (*2+1)
// Curve No 9: RPM      -> 18 (*2)
// Curve No 9: Degrees  -> 19 (*2+1)
// Curve No 10: RPM      -> 20 (*2) // Last RPM (numer depending on number of points) is fixated on 8000 RPM
// Curve No 10: Degrees  -> 21 (*2+1)
uint32_t TuneGraphNo2decimal(uint8_t MSB, uint8_t LSB)
{
  uint32_t value;
  char str[3];      // char string for hex number

  str[0]=MSB;     // fill string, to convert a single hex char
  str[1]=LSB;
  str[2]='\0';
  value=strtol(str,NULL,16);
  
  
return(value);
}




// returns sigend int (-20 -100) in celcius
int32_t TuneTemperature2decimal(uint8_t MSB, uint8_t LSB)
{
  uint32_t value;
  char str[3];      // char string for hex number

  str[0]=MSB;     // fill string, to convert a single hex char
  str[1]=LSB;
  str[2]='\0';
  value=strtol(str,NULL,16);
  
  value-=30;      // emperisch found offset correction value 0 is; -25 on gauge 10 -> -20 

return(value);
}




// converts bluetooth binary to advance in degrees
double TuneAdvance2decimal(uint8_t MSB, uint8_t LSB)
{
  uint32_t MSB_dec;
  uint32_t LSB_dec;

  double value;
  double MSB_weight=3.2/1;        // From readout digital advance curve Scaling MSB != scaling LSB   
  double LSB_Weight=0.2/1;         // From roudaout advance curve, is not exact due to jump in readout table. 0,2 degrees == 1 bit
  char str[2];                    // char string for hex number

  str[0]=MSB;                     // fill string, to convert a single hex char
  str[1]='\0';
  MSB_dec=strtol(str,NULL,16);

  str[0]=LSB;                     // fill string, to convert a single hex char
  str[1]='\0';
  LSB_dec=strtol(str,NULL,16);
  
  value=MSB_dec*MSB_weight + LSB_dec*LSB_Weight;

return(value);
}


// converst hex value as string to integer (0-0xFF)
// Same logic as TunePinCode2char
uint32_t TunePressure2decimal(uint8_t MSB, uint8_t LSB)
{
  uint32_t value;
  char str[3];      // char string for hex number

  str[0]=MSB;     // fill string, to convert a single hex char
  str[1]=LSB;
  str[2]='\0';
  value=strtol(str,NULL,16);
  
  
return(value);
}



// converts a single Tune Char into Pincode Char ('0'-'9')
// Logic same as TunePressure2decimal(), except output is presented as char
char TunePinCode2char(uint8_t MSB, uint8_t LSB)
{
        uint32_t value;
        char str[3];                    // char string for hex number

        str[0]=MSB;                     // fill string, to convert a single hex char
        str[1]=LSB;
        str[2]='\0';
        value=strtol(str,NULL,16);

        if(!isdigit(value))  {
          printf("%s Conversion Error, result is not a char: %02X\n",__FUNCTION__, value);
          return(-1);
  }

return(value);
}


// Calculates and sets checksum byte [3]
// see checksum.txt for finding/logic deduction
// csum should point to byte number 3. this construciton is to prevent writing other bytes
uint32_t TuneSetnewChecksum(const uint8_t* str,uint8_t *csum)
{

//CSUM= <ID>+0x10 + <MSB-0x30) + (LSB-0x30)
// write byte 3 in str
csum[0]=str[0]+0x10 + str[1]-'0' + str[2]-'0';


return(0);
}

// in kP / absolute
// values returned in param 2 and 3
// same logic as char2TunePinCode()
uint32_t decimal2TunePressure(uint32_t Pressure, uint8_t *MSB, uint8_t *LSB)
{
  char str_m[20];     // char string for hex number

  
  sprintf(str_m,"%02X",Pressure);   // No rounding, exact
  MSB[0]=str_m[0];
  LSB[0]=str_m[1];

  //printf("%s pressure=%d kP   MSB=%c LSB=%c str_m=%s\n",__FUNCTION__, Pressure,  MSB[0],LSB[0],  str_m);
  
return(0);
}


double TuneVoltage2decimal(uint8_t MSB, uint8_t LSB)
{
  double value;
  double WeightFactor=0x40/14.1;    // op basis van afgelezen waardes
  char str[3];      // char string for hex number

  str[0]=MSB;     // fill string, to convert a single hex char
  str[1]=LSB;
  str[2]='\0';
  value=strtol(str,NULL,16);
  
  value/=WeightFactor;      // scaling factor, emperisch

return(value);
}


double TuneAmpere2decimal(uint8_t MSB, uint8_t LSB)
{
  double value;
  double WeightFactor=16/1.85;    // op basis van afgelezen waardes
  char str[3];      // char string for hex number

  str[0]=MSB;     // fill string, to convert a single hex char
  str[1]=LSB;
  str[2]='\0';
  value=strtol(str,NULL,16);
  
  value/=WeightFactor;      // scaling factor, emperisch

return(value);
}


// retuns the actual value of Advance based on a number of paramters
// MUST call this function before reading/transmitting/displaying values
 //server->tune_Pressure
 //server->tune_RPM
 //server->tune_AdvanceCurveRPM[]
 //server->tune_MapCurveDegrees[]
 //server->tune_TuningMode_Advance
 //server->tune_MapCurveStartRPM
 //server->tune_RPMLimit
 //
void UpdateRealtimeTuneAdvance(void *user_data)
{
  //struct server *server = user_data;
  server_s_t* server=(server_s_t*)&user_data;
  double Advance;


  // Calculate base Advance based on RPM/Advance graph
  Advance=CalculateAdvanceByRPM(server);

  // Most simplistic for now
  // read current value
  //Advance=TuneAdvance2decimal(server->tune_Advance[0],server->tune_Advance[1]); 
  
  // Add the Advance based on Vacuum/Advance Graph
  Advance+=CalculateAdvanceByPressure(server);

  // Tuning offset, (when in Tuning mode only!)
  if(server->tune_TuningMode_enabled) {
    Advance+=server->tune_TuningMode_Advance;
  }

  // write back updated value
  decimal2TuneAdvance(Advance, &server->tune_Advance[0], &server->tune_Advance[1]);
}


// Turns decimal bluetooth binary bluetooth binary
// returns values in value pointers MSB, LSB
uint32_t decimal2TuneRPM(uint32_t RPM, uint8_t *MSB, uint8_t *LSB)
{
  uint32_t MSB_dec;
  uint32_t LSB_dec;
  uint32_t MSB_weight=800/1;  // From readout digital advance curve Scaling MSB != scaling LSB
  uint32_t LSB_Weight=50/1;   // From roudaout advance curve, is not exact due to jump in readout table.
  char str_m[20];     // char string for hex number
  char str_l[20];     // char string for hex number

  MSB_dec=RPM/MSB_weight;   // div/800
  LSB_dec=(RPM%MSB_weight)/LSB_Weight;  // rest div/50 (no rounding required, accuracy is 50RPM, rounded to below.
  
  sprintf(str_m,"%X",MSB_dec);
  MSB[0]=str_m[0];

  
  sprintf(str_l,"%X",LSB_dec);
  LSB[0]=str_l[0];

  //printf("%s MSB_dec=%d LSB_dec=%d   MSB=%c LSB=%c str_m=%s str_l=%s\n",__FUNCTION__,  MSB_dec,LSB_dec,  MSB[0],LSB[0],  str_m,str_l);
  
return(0);
}

uint32_t decimal2TuneAmpere(double Ampere, uint8_t *MSB, uint8_t *LSB)
{
  char str_m[20];     // char string for hex number
  
  double WeightFactor=16/1.85;  // op basis van afgelezen waardes
  uint32_t A=ROUND_2_INT(Ampere*WeightFactor);    // round instead chop

  sprintf(str_m,"%02X",A);
  MSB[0]=str_m[0];
  LSB[0]=str_m[1];

  //printf("%s Ampere=%d A   MSB=%c LSB=%c str_m=%s\n",__FUNCTION__, A,  MSB[0],LSB[0],  str_m);
  
return(0);
}

// Volt graphs is very course, and not recommended for anything more percise then 0.5V!!!!
// This is due to non optimal value/scale is used, and one bit ~= 0.3V !!!!
uint32_t decimal2TuneVoltage(double Voltage, uint8_t *MSB, uint8_t *LSB)
{
  char str_m[20];     // char string for hex number
  
  //double WeightFactor=4*14.0/12.28;   // op basis van afgelezen waardes
  double WeightFactor=0x40/14.1;      // op basis van 0x24. 00 ~= 14,1V sample ((maar is aan lage kant afgerond door bit granulariteit
  uint32_t Volt=ROUND_2_INT(Voltage*WeightFactor);  

  sprintf(str_m,"%02X",Volt);
  MSB[0]=str_m[0];
  LSB[0]=str_m[1];

  //printf("%s Volt=%d V (123Tune)  MSB=%c LSB=%c str_m=%s\n",__FUNCTION__, Volt,  MSB[0],LSB[0],  str_m);
  
return(0);
}


// -20 ... 100 degress celcius
// offset of 30, in celsius, and linear in celsius.
uint32_t decimal2TuneTemperature(double Temperature, uint8_t *MSB, uint8_t *LSB)
{
  char str_m[20];     // char string for hex number
  
  uint32_t T=ROUND_2_INT(Temperature+30); // emperisch found offset correction value 0 is; -25 on gauge 10 -> -20. use rounding instead chopping (but more accuracy then 1 degree is not usefull)

  sprintf(str_m,"%02X",T);
  MSB[0]=str_m[0];
  LSB[0]=str_m[1];

  //printf("%s Temperature=%d    MSB=%c LSB=%c str_m=%s\n",__FUNCTION__, T,  MSB[0],LSB[0],  str_m);
  
return(0);

}


// float like 20.0 may be actually 19.999999999, and therefore the actual caclulated value may get chopped off to 19.8 (due to bit persision)
// the persision is 0.2 degrees, so adding 0.1 gets rounding fixed for positive numbers.
uint32_t decimal2TuneAdvance(double Advance, uint8_t *MSB, uint8_t *LSB)
{
  uint32_t MSB_dec;
  uint32_t LSB_dec;
  double MSB_weight=3.2/1;  // From readout digital advance curve Scaling MSB != scaling LSB
  double LSB_Weight=0.2/1;    // From roudaout advance curve, is not exact due to jump in readout table.
  char str_m[20];     // char string for hex number
  char str_l[20];     // char string for hex number

  Advance+=0.001; //helps natural rounding for positive numbers
  MSB_dec=Advance/MSB_weight;   // div/800
  LSB_dec=(Advance-MSB_dec*MSB_weight)/LSB_Weight;  // rest div/50
  
  sprintf(str_m,"%X",MSB_dec);
  MSB[0]=str_m[0];

  
  sprintf(str_l,"%X",LSB_dec);
  LSB[0]=str_l[0];

  //printf("%s MSB_dec=%d LSB_dec=%d   MSB=%c LSB=%c str_m=%s str_l=%s\n",__FUNCTION__,  MSB_dec,LSB_dec,  MSB[0],LSB[0],  str_m,str_l);
  
return(0);

}


// Calcualte Advance based on current Pressure and Pressure(Vacuum)/Advance Graph

double CalculateAdvanceByPressure(void *user_data)
{
//  struct server *server = user_data;
  server_s_t* server=(server_s_t*)&user_data;
  
  uint32_t Pressure=TunePressure2decimal(server->tune_Pressure[0],server->tune_Pressure[1]);
        uint32_t Lower_Pressure=0, Upper_Pressure;  // Graphs points
        int i;
        uint32_t RPM;
        uint32_t MapCurveStartRPM;
        
        RPM=TuneRPM2decimal(server->tune_RPM[0],server->tune_RPM[1]);
        MapCurveStartRPM=TuneRPM2decimal(server->tune_MapCurveStartRPM[0],server->tune_MapCurveStartRPM[1]);
        
        if(RPM < MapCurveStartRPM) {
          printf("%s MAPCurve not active RPM<MapCurveStartRPM (%d<%d) adding 0 degrees\n",__FUNCTION__,RPM,MapCurveStartRPM);
          
          return(0);
        }
        


        


  // calculate number of elements first!!!! <TODO>
  for(i=0;i<(10-1);i++) { // 10-1 due to number of zones insted of boundaries
    Upper_Pressure=TunePressure2decimal(server->tune_MapCurvePressure[i][0],server->tune_MapCurvePressure[i][1]);
    
    if(i==0 && Pressure < Upper_Pressure) { // TODO <= ??
      printf("%s MAPCurve before first point adding 0 degrees\n",__FUNCTION__);
      return(0);  // 0 degrees or inactive until first point on the map
    }
    
    // between these points?
    if((Lower_Pressure <= Pressure) && (Pressure <= Upper_Pressure) ) {
      double Lower_Advance, Upper_Advance, Advance;
    
      // linear interpolation MAP graph
      Upper_Advance=TuneAdvance2decimal(server->tune_MapCurveDegrees[i][0],  server->tune_MapCurveDegrees[i][1]); 

      if (i==0) { // prevent i-1=-1 situation while startup for out of range array.
        // i=0, i-1 <0
        Lower_Advance = Upper_Advance;
      } else {
        //  i-1>=0
        Lower_Advance=TuneAdvance2decimal(server->tune_MapCurveDegrees[i-1][0],server->tune_MapCurveDegrees[i-1][1]);
      }
      
      // cast to double to force franctional devision 
      // Prevent devision by zero if Upper_Pressure-Lower_Pressure==0
      if( (Upper_Pressure - Lower_Pressure) ==0) {
        // Upper_Pressure == Lower_Pressure
        Advance =  Lower_Advance;   // or Upper_Advance, which is the same in this case.
      } else {  
        // Upper_Pressure != Lower_Pressure
        Advance = ((double)(Pressure - Lower_Pressure)/(Upper_Pressure - Lower_Pressure)) * (Upper_Advance - Lower_Advance) + Lower_Advance;
      }

      // wrong, but fot testing simpel just send UPPER boundary value.... not the linear calculated value. <TODO>
      //return(TuneAdvance2decimal(server->tune_MapCurveDegrees[i][0],server->tune_MapCurveDegrees[i][1]));

      printf("%s MAPCurve is active RPM>=MapCurveStartRPM (%d>=%d) adding %f degrees\n",__FUNCTION__,RPM,MapCurveStartRPM, Advance);

      return(Advance);
      }
    
    // prepare for next round
    Lower_Pressure=Upper_Pressure;
    }
  

  

// dummy to build framework <TODO>
// 0 when not found, or unforseed/undefined
printf("%s pressure %d not found in graph\n",__FUNCTION__,Pressure);
return(0);
}






// Calcualte Advance based on current RPM and RPM/Advance Graph
double CalculateAdvanceByRPM(void *user_data)
{
//  struct server *server = user_data;
  server_s_t* server=(server_s_t*)&user_data;
  
  uint32_t RPM=TuneRPM2decimal(server->tune_RPM[0],server->tune_RPM[1]);
  uint32_t Lower_RPM=0, Upper_RPM;  // Graph points
  int i;
  

  // calculate number of elements first!!!! <TODO>
  for(i=0;i<(10-1);i++) { // 10-1 due to number of zones insted of boundaries
    Upper_RPM=TuneRPM2decimal(server->tune_AdvanceCurveRPM[i][0],server->tune_AdvanceCurveRPM[i][1]);
    
    if(i==0 && RPM < Upper_RPM) { // TODO <= ??
      printf("%s AdvanceCurve before first point adding 0 degrees\n",__FUNCTION__);
      return(0);  // 0 degrees or inactive until first point on the map
    }
    
    // between these points?
    if((Lower_RPM <= RPM) && (RPM <= Upper_RPM) ) {
      double Lower_Advance, Upper_Advance, Advance;
  
      // linear interpolation
      Upper_Advance=TuneAdvance2decimal(server->tune_AdvanceCurveDegrees[i][0],  server->tune_AdvanceCurveDegrees[i][1]); 
      if (i==0) {     // prevent i-1=-1 situation while startup for out of range array.
        // i=0, i-1 <0
                                Lower_Advance = Upper_Advance;
      } else {  
        Lower_Advance=TuneAdvance2decimal(server->tune_AdvanceCurveDegrees[i-1][0],server->tune_AdvanceCurveDegrees[i-1][1]);
      }
      
      if ( (Upper_RPM - Lower_RPM) == 0) {
        // equal, prevend deviding by zero
        Advance = Lower_Advance;  // or Upper_Advance, which is the same
      } else {
        // cast to double to force franctional devision 
        Advance = ((double)(RPM-Lower_RPM)/(Upper_RPM-Lower_RPM)) * (Upper_Advance - Lower_Advance) + Lower_Advance;
      }
  
  
    
      // wrong, but fot testing simpel just send UPP boundary value.... not the linear calculated value. <TODO>
      //return(TuneAdvance2decimal(server->tune_AdvanceCurveDegrees[i][0],server->tune_AdvanceCurveDegrees[i][1]));

      printf("%s AdvanceCurve is active adding %f degrees\n",__FUNCTION__, Advance);

      return(Advance);
      }
    
    // prepare for next round
    Lower_RPM=Upper_RPM;
    }
  
  

// dummy to build framework <TODO>
// 0 when not found, or unforseed/undefined
printf("%s RPM %d not found in graph\n",__FUNCTION__,RPM);
return(0);
}



// Logic same as decimal2TunePressure(), except input is presented as char
uint32_t char2TunePinCode(char PinChar, uint8_t *MSB, uint8_t *LSB)
{
        char str_m[20];                 // char string for hex number


        sprintf(str_m,"%02X",PinChar);         // No rounding, exact
        MSB[0]=str_m[0];
        LSB[0]=str_m[1];

        //printf("%s pressure=%d kP   MSB=%c LSB=%c str_m=%s\n",__FUNCTION__, Pressure,  MSB[0],LSB[0],  str_m

return(0);

}
