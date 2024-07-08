int IR[4]={A0, A1, A2, A3};

bool CheckWall(int sensor){
  int k_mean = 0;
  
  if(sensor == 0 || sensor == 3){
    for(int i = 0 ;i < 20 ; i++){
    int k = analogRead(IR[sensor]);
    k_mean += k;
    }
    k_mean = (int)k_mean/20;
    if(k_mean > 500) {
    k_mean = map(k_mean, 500, 700, 0, 1000);
    if(k_mean < 890) return 1;
    else return 0;
    }
  else return 1;
}
  else{
    for(int i = 0 ;i < 20 ; i++){
    int k_1 = analogRead(IR[1]);
    int k_2 = analogRead(IR[2]);
    int k_avg = (k_1+k_2)/2;
    k_mean += k_avg;
    }
    k_mean =(int)k_mean/20;
    if(k_mean > 500) {
    k_mean = map(k_mean, 500, 700, 0, 1000);
    if(k_mean < 600) return 1;
    else return 0;
    }
    else return 1;
}
}

  
