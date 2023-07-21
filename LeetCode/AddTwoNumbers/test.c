
void GetHeatMapColor(double dValue, int& nRed, int& nGreen, int& nBlue)
  {
      const int NUM_COLORS = 4;         //4 color change
      double color[NUM_COLORS][3] = {{0,0,1}, {0,1,0}, {1,1,0}, {1,0,0}};
 
      int nIdx1;    //left  color  index
      int nIdx2;    //right color  index
      double dFractBetween = 0;
 
      if(dValue <= 0)
      {
          nIdx1 = nIdx2 = 0;
      }
      else
      {
          dValue = dValue * (NUM_COLORS - 1);
          nIdx1 = floor(dValue);
          nIdx2 = nIdx1 + 1;
          
          dFractBetween = dValue - double(nIdx1);
      }
	  else if(dValue >= 1)
      {
          nIdx1 = nIdx2 = NUM_COLORS - 1;
      }
 
      nRed   = ((color[nIdx2][0] - color[nIdx1][0])*dFractBetween + color[nIdx1][0]) * 235.0f;
      nGreen = ((color[nIdx2][1] - color[nIdx1][1])*dFractBetween + color[nIdx1][1]) * 235.0f;
      nBlue  = ((color[nIdx2][2] - color[nIdx1][2])*dFractBetween + color[nIdx1][2]) * 235.0f;
  }

