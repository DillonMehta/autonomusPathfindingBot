String maze[4][4] = {
  { "", "EBR", "GLR", "L" },
  { "B", "T", "B", "" },
  { "GT", "BR", "TL", "GB" },
  { "", "ST", "", "T" }
};
//Adjacency Matrix
int matrix[16][16];



void createMatrix(){
    //EMPTY MATRIX:
    for (int a = 0; a < 16; a++) {
      for (int b = 0; b < 16; b++) {
        matrix[a][b] = 0;
      }
    }
  //OVERWRITING VALUES   
  for (int a = 0; a < 16; a++) {
    int i = a / 4;
    int j = a % 4;
    String str = maze[i][j];
    int w = -1;//DEFAULT CONNECTION WEIGHT
    if (str.indexOf('G') >= 0) { //IF ITS A GATE ZONE
      w = -16; //GATE ZONE CONNECTION WEIGHT
    }
    if (str.indexOf('T') < 0 && i > 0){ //IF THERE IS NO TOP BARRIER && IS NOT IN THE TOP ROW
      matrix[4 * (i - 1) + j][4 * i + j] = w;
      if(maze[i-1][j].indexOf('G') >= 0){//CHECK IF IT LEADS TO A GATE ZONE
        matrix[4 * (i - 1) + j][4 * i + j] = -16;
      }
    }
    if (str.indexOf('B') < 0 && i < 3) {// IF THERE IS NO BOTTOM BARRIER && IS NOT IN THE BOTTOM ROW      
      matrix[4 * (i + 1) + j][4 * i + j] = w;
      if(maze[i+1][j].indexOf('G') >= 0){//CHECK IF IT LEADS TO A GATE ZONE
        matrix[4 * (i + 1) + j][4 * i + j] = -16;
      }
    } 
    if (str.indexOf('L') < 0 && j > 0 ) {//IF THERE IS NO LEFT BARRIER && IS NOT IN THE LEFT-MOST COLUMN
      matrix[4 * i + j][4 * i + j - 1] = w;
      if(maze[i][j-1].indexOf('G') >= 0){//CHECK IF IT LEADS TO A GATE ZONE
        matrix[4 * i + j][4 * i + j - 1] = -16;
      }
    }
    if (str.indexOf('R') < 0 && j < 3) { // IF THERE IS NO RIGHT BARRIER && IS NOT IN THE RIGHT MOST COLUMN
      matrix[4 * i + j][4 * i + j + 1] = w;
      if(maze[i][j+1].indexOf('G') >= 0){//CHECK IF IT LEADS TO A GATE ZONE
        matrix[4 * i + j][4 * i + j + 1] = -16;
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  createMatrix();
  //BARRIERS- 
  // B: Bottom 
  // T: Top 
  // R: Right 
  // L: Left
  //G = Gate Zone 
  //S = Start
  //E = End



  // Printing matrix
  delay(1000);
  Serial.println(" ");
  for (int a = 0; a < 16; a++) {
    for (int b = 0; b < 16; b++) {
      Serial.print(matrix[a][b]);
      Serial.print("\t");
    }
    Serial.println(" ");
  }


}

void loop() {
  // put your main code here, to run repeatedly:
}
