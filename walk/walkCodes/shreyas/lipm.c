void ssp(float ssptime)
{

	float wtShift = 30;
	float startZ  = -acosh(wtShift); 
	float stopZ   = acosh(wtShift);
	
	for(walkTime=0;walkTime<=sspTime;walkTime+=1.0/fps)
	{
		x  = height-(lift)*(sin(xfreq*(walkTime-startX)/(stopX-startX)+xPhase)+displacement)/(1+displacement);
		xr = height;

		y=
		yr=-(suplegYin*cosh(walkTime*gravConstant)+veloYin*sinh(walkTime*gravConstant)/gravConstant);
		
		z  = 10+(wtShift-cosh(linear(startZ,stopZ,walkTime,sspTime)));
		zr = -z;
	}
}

void dsp(float dsptime)
{
	veloZ=20.0/dsptime;
	for(walkTime=0;walkTime<=dspTime;walkTime+=1.0/fps)
	{
		x=height;
		xr=height;
		
		y  = y - veloY/fps;
		yr = yr - veloY/fps;
	
		z=10-veloZ*walkTime;
		zr=-z;
	}	
