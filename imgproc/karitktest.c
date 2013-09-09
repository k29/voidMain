void FeatureDetection::getInGreenKartik(CamCapture &cam)
{
	// Make reduced image
	int rowsum[IMAGE_HEIGHT/4];
	IplImage* binary_image = cvCreateImage(cvSize(IMAGE_WIDTH/4, IMAGE_HEIGHT/4), 8, 1);
	//IplImage* prob_image = cvCreateImage(cvSize(IMAGE_WIDTH/4, IMAGE_HEIGHT/4), 8, 1);
	IplImage* show_image = cvCreateImage(cvSize(IMAGE_WIDTH*2, IMAGE_HEIGHT*2), 8, 1);
	//cvZero(seg_white);
	//cvZero(seg_black);
	//cvZero(seg_green);
	//cvZero(seg_white_count);
	//const int tmin = 27;
 	//const int trow = IMAGE_WIDTH*18/4;
 	//const int tsum = 36*18;
 	for(int y = 0; y < IMAGE_HEIGHT/4; y++)
 	{
 		rowsum[y] = 0;
 		for(int x = 0; x < IMAGE_WIDTH/4; x++)
 		{
 			int wcount = 0;
 			int gcount = 0;
 			int bcount = 0;
 			int rcount = 0;
 			int ngreen=0;
 			int nred=0;
 			for(int xx = 0; xx < 4; xx++)
 			for(int yy=0; yy < 4; yy++)
 			{
 				int tx = x*4 + xx;
 				int ty = y*4 + yy;
 				if(cam.isGreen_small(tx, ty))
 					gcount++;
 		                if(cam.isRed_small(tx, ty))
 					rcount++;
 				if(cam.isWhite_small(tx	, ty))
 					wcount++;
 				if(wcount>5 && gcount>5 || rcount>4)
 				returnPixel1C(binary_image,x,y)=255;
 				else if(wcount && gcount>9 || rcount>7)
 				returnPixel1C(binary_image,x,y)=255;	
 					
 					/*if(tx+35<IMAGE_WIDTH && tx-35>0 && ty+35<IMAGE_HEIGHT && ty-35>0)
 						{
 		
 						for(int i=-35;i<35;i++)
 						for(int j=-35;j<35;j++)
 							{
 							if(cam.isGreen_small(tx+i,ty+j))
 							ngreen++;
 							if(cam.isRed_small(tx+i,ty+j))
 							nred++;
 							}
 						}
 					else if(tx+35>IMAGE_WIDTH || tx-35<0 || ty+35>IMAGE_HEIGHT || ty-35<0)
 						{
 						int a,b;
 						int k,p;
 						if(tx+35>IMAGE_WIDTH || tx-35<0)
 							{
 							if(tx+35>IMAGE_WIDTH)
 								{
 								k=IMAGE_WIDTH-tx;
 								a=k-70;
 								}
 							else 
 								{
 								a=-1*tx;
 								k=70+a;
 								}
 							}
 						else
 							{
 							
 							if(ty+35>IMAGE_HEIGHT)
 								{
 								p=IMAGE_HEIGHT-ty;
 								b=p-70;
 								}
 							else
 								{
 								b=-1*ty;
 								p=70+b;
 								}
 							}
 							
 						for( int r=a;r<k;r++)
 						for(int s=b;s<p;s++)
 							{
 							if(cam.isGreen_small(tx+r,ty+s))
 							ngreen++;
 							if(cam.isRed_small(tx+r,ty+s))
 							nred++;
 							}
 						}	
 					if(ngreen>1200)
 					returnPixel1C(binary_image, x, y)=255;
 					else if(ngreen>500 && nred>700)
 					returnPixel1C(binary_image, x, y)=255;	
 					nred=0;
 					ngreen=0;
 						
 					}	*/
 							
 				//cvShowImage("binary" , binary_image);
 				/*cvResize(binary_image, show_image);
	                        cvShowImage("Binary", show_image);	
	                        cvReleaseImage(&show_image);

	//Segmentation done
	                        cvReleaseImage(&binary_image);	*/
	                        //IplImage* show_image = cvCreateImage(cvSize(IMAGE_WIDTH*2, IMAGE_HEIGHT*2), 8, 1);
 				//cvResize(binary_image, show_image);
	                        cvShowImage("Binary", binary_image);	
	                        cvWaitKey(1);
	                        //cvReleaseImage(&show_image);
                            
	//Segmentation done
	                        //cvReleaseImage(&binary_image);	
 								
 						        
 					
 			}
 		}
 		     //cvReleaseImage(&show_image);          
 								
	}		
 cvReleaseImage(&show_image); 	
 }


