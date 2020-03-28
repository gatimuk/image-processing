static void mv_func(MpegEncContext *s, int *mbx, int *mby, int *mv_func_x, int *mv_func_y, int *mv_func_ref, uint8_t *MB_status, uint8_t *fixed, int *avgDist, int *pass){
	#define MB_EC 1 //MB lost; error concealment required
	#define MB_OK 2 //MB OK; no error concealment applied
    #define MV_FROZEN 3
    #define MV_CHANGED 2
    //const int mb_stride = s->mb_stride;
    //const int mb_width = s->mb_width;
    //const int mb_height= s->mb_height;
	int mb_x = *mbx;
	int mb_y = *mby;
    int mot_step=0;
	int mot_stride=0;
	
	int MB_index_up, MB_index_dn, up, dn; //indices for MBs (up/dn) used for co-location
	int cluster[12][4]={{0}};
	//[8][]: left, right, top/bottom -> X2 -> for both top and bottom slices
    //[+4][]: top, bottom, left, right MV estimates for fine-tuning
	//[][4]: MBxy, MVx, MVy, MVref
	float cluster2[12];
	//Inverse distance weights for cluster[][]
	int MV_index; //MV index (mot_index) of candidate MV (MV_up or MV_dn)
	int step=0; //step counter for searching neighboring inter-MBs for generating MV_wt
	float MVx_avg = 0.0, MVy_avg = 0.0, MV_ref_avg = 0.0; //MV weighted averages
	//float MV_up_mag, MV_dn_mag; //MV

	int i;
	
	//Placeholders for mb_x and mb_y
	int temp_x = 0;
	int temp_y = 0;
	
	//Sum of distances of valid MVs from lost MV. Aids in calc of avg dist
	float sumDist = 0;
    
    const int mb_xy= mb_x + mb_y * s->mb_stride;
	const int error= s->error_status_table[mb_xy];
    
    set_mv_strides(s, &mot_step, &mot_stride);

	//If MB is OK, then continue (skip to next MB)
	if(IS_INTRA(s->current_picture.f.mb_type[mb_xy]) && !(error&(AC_ERROR|DC_ERROR|MV_ERROR)))
		return;
	//Added this: If it's intra anyway, return.
	if(IS_INTRA(s->current_picture.f.mb_type[mb_xy]))
		return;
	if(!(error&MV_ERROR)) return;
	
	//If motion vector (MV) is lost
	if(!IS_INTRA(s->current_picture.f.mb_type[mb_xy]) && error&(AC_ERROR|DC_ERROR|MV_ERROR)) {
		//Find co-located edge MVs (of intact top and bottom slices)
		//If edge MB is not inter, keep searching up/down.
		int count = 0;
		float sum_x = 0.0, sum_y = 0.0, sum_denom = 0.0;
        int sum_ref = 0;
        const int mot_index= (mb_x + mb_y*mot_stride) * mot_step;
        
		//*****
		//Top cluster
		//*****
		up = 1;
		MB_index_up = mb_xy - up * s->mb_stride;
		while(IS_INTRA(s->current_picture.f.mb_type[MB_index_up]) || MB_status[MB_index_up] == MB_EC) {
			up += 1;
			MB_index_up = mb_xy - (up * s->mb_stride);
			if(MB_index_up < 0) {
				goto bottom;
			}
		}
		MV_index = mot_index - mot_stride*mot_step*up;
		
		cluster[count][0] = MB_index_up;
		cluster[count][1] = s->current_picture.f.motion_val[0][MV_index][0];
		cluster[count][2] = s->current_picture.f.motion_val[0][MV_index][1];
		cluster[count][3] = s->current_picture.f.ref_index[0][4*MB_index_up];
		cluster2[count] = 1/(float)up;
		
		count += 1;

		//Find MVs from neighboring inter-MBs for weighting
		//Each inter-MB neighbor is "step" units away from MB_index and its MV is weighted as MV*(1/step)
        
		//Left neighbor
		if(mb_x>0) {
			temp_x = mb_x;
			step = 1;
			cluster[count][0] = MB_index_up - step;
			while(IS_INTRA(s->current_picture.f.mb_type[cluster[count][0]]) || MB_status[cluster[count][0]] == MB_EC) {
				step += 1;
				temp_x = temp_x - step;
				if(temp_x < 0) {
					goto right_top;
				}
				cluster[count][0] = MB_index_up - step;
			}
			cluster[count][1] = s->current_picture.f.motion_val[0][MV_index - mot_step*step][0];
			cluster[count][2] = s->current_picture.f.motion_val[0][MV_index - mot_step*step][1];
			cluster[count][3] = s->current_picture.f.ref_index[0][4*(cluster[count][0])];
			cluster2[count] = 1/(float)pow((pow(up,2)+pow(step,2)),0.5);
			
			count += 1;
		}
		
		//Right neighbor
		right_top:
		if(mb_x+1<s->mb_width) {
			temp_x = mb_x;
			step = 1;
			cluster[count][0] = MB_index_up + step;
			while(IS_INTRA(s->current_picture.f.mb_type[cluster[count][0]]) || MB_status[cluster[count][0]] == MB_EC) {
				step += 1;
				if (temp_x > 119) {
					goto top_top;
				}
				cluster[count][0] = MB_index_up + step;
			}
			cluster[count][1] = s->current_picture.f.motion_val[0][MV_index + mot_step*step][0];
			cluster[count][2] = s->current_picture.f.motion_val[0][MV_index + mot_step*step][1];
			cluster[count][3] = s->current_picture.f.ref_index[0][4*(cluster[count][0])];
			cluster2[count] = 1/(float)pow((pow(up,2)+pow(step,2)),0.5);
			
			count += 1 ;
		}
		
		//Top neighbor
		top_top:
		if(MB_index_up>119) {
			temp_y = mb_y - up;
			step = 1;
			cluster[count][0] = MB_index_up - step * s->mb_stride;
			while(IS_INTRA(s->current_picture.f.mb_type[cluster[count][0]]) || MB_status[cluster[count][0]] == MB_EC) {
				step += 1;
				temp_y = temp_y - step;
				if (temp_y < 0) {
					goto bottom;
				}
				cluster[count][0] = MB_index_up - step * s->mb_stride;
			}
			cluster[count][1] = s->current_picture.f.motion_val[0][MV_index - mot_stride*mot_step*step][0];
			cluster[count][2] = s->current_picture.f.motion_val[0][MV_index - mot_stride*mot_step*step][1];
			cluster[count][3] = s->current_picture.f.ref_index[0][4*(cluster[count][0])];
			cluster2[count] = 1/(float)(up+step);
			
			count +=1;
		}
		
		
		//*****
		//Bottom cluster
		//*****
		bottom:
		dn = 1;
		MB_index_dn = mb_xy + dn * s->mb_stride;
		while(IS_INTRA(s->current_picture.f.mb_type[MB_index_dn]) || MB_status[MB_index_dn] == MB_EC) {
			dn += 1;
			MB_index_dn = mb_xy + (dn * s->mb_stride);
			if(MB_index_dn > 8228) {
				goto calc;
			}
		}
		MV_index = mot_index + mot_stride*mot_step*dn;
		
		cluster[count][0] = MB_index_dn;
		cluster[count][1] = s->current_picture.f.motion_val[0][MV_index][0];
		cluster[count][2] = s->current_picture.f.motion_val[0][MV_index][1];
		cluster[count][3] = s->current_picture.f.ref_index[0][4*MB_index_dn];
		cluster2[count] = 1/(float)dn;
        
		count += 1 ;
		
		//Find neighboring inter-MBs for weighting
		//Left
		if(mb_x>0) {
			temp_x = mb_x;
			step = 1;
			cluster[count][0] = MB_index_dn - step;
			while(IS_INTRA(s->current_picture.f.mb_type[cluster[count][0]]) || MB_status[cluster[count][0]] == MB_EC) {
				step += 1;
				temp_x = temp_x - step;
				if (temp_x < 0) {
					goto right_bottom;
				}
				cluster[count][0] = MB_index_dn - step;
			}
			cluster[count][1] = s->current_picture.f.motion_val[0][MV_index - mot_step*step][0];
			cluster[count][2] = s->current_picture.f.motion_val[0][MV_index - mot_step*step][1];
			cluster[count][3] = s->current_picture.f.ref_index[0][4*(cluster[count][0])];
			cluster2[count] = 1/(float)pow((pow(dn,2)+pow(step,2)),0.5);
			
			count += 1;
		}
		
		//Right
		right_bottom:
		if(mb_x+1<s->mb_width) {
			temp_x = mb_x;
			step += 1;
			temp_x = temp_x + step;
			if (temp_x > 119) {
				goto bottom_bottom;
			}
			cluster[count][0] = MB_index_dn + step;
			while(IS_INTRA(s->current_picture.f.mb_type[cluster[count][0]]) || MB_status[cluster[count][0]] == MB_EC) {
				step++;
				cluster[count][0] = MB_index_dn + step;
			}
			cluster[count][1] = s->current_picture.f.motion_val[0][MV_index + mot_step*step][0];
			cluster[count][2] = s->current_picture.f.motion_val[0][MV_index + mot_step*step][1];
			cluster[count][3] = s->current_picture.f.ref_index[0][4*(cluster[count][0])];
			cluster2[count] = 1/(float)pow((pow(dn,2)+pow(step,2)),0.5);
			
			count += 1 ;
		}
		
		//Bottom
		bottom_bottom:
		if(MB_index_dn<8107) {
			temp_y = mb_y + dn;
			step = 1;
			cluster[count][0] = MB_index_dn + step * s->mb_stride;
			while(IS_INTRA(s->current_picture.f.mb_type[cluster[count][0]]) || MB_status[cluster[count][0]] == MB_EC) {
				step += 1;
				temp_y = temp_y + step;
				if (temp_y > 67) {
					goto calc;
				}
				cluster[count][0] = MB_index_dn + step * s->mb_stride;
			}
			cluster[count][1] = s->current_picture.f.motion_val[0][MV_index + mot_stride*mot_step*step][0];
			cluster[count][2] = s->current_picture.f.motion_val[0][MV_index + mot_stride*mot_step*step][1];
			cluster[count][3] = s->current_picture.f.ref_index[0][4*((int)cluster[count][0])];
			cluster2[count] = 1/(float)(dn+step);
			
			count +=1;
		}
		
		calc:
		if(count==0) return;
		for(i=0; i<count; i++) {
			sum_x+= (float)cluster[i][1] * cluster2[i];
			sum_y+= (float)cluster[i][2] * cluster2[i];
			sum_ref+= cluster[i][3];
			sum_denom+= cluster2[i];
		}
        
        // Calc avg dist of valid MVs from lost MV
		for(i=0; i<count; i++) {
			temp_y = cluster[count][0]/s->mb_stride;
			temp_x = cluster[count][0]%s->mb_stride;
			sumDist = sumDist + pow((pow((temp_x-mb_x),2)+pow((temp_y-mb_y),2)),(0.5));
		}
		
		*avgDist = (int)(sumDist/count);
        
        #if 0
        av_log(s->avctx, AV_LOG_DEBUG, "mv_func: MB to be estimated = (%d,%d: %d)\n", mb_x, mb_y, mb_xy);
        //Fine-tuning with MV estimates (left, right, top, bottom)
        if(*pass>=1) {
            //Left estimate
            if (mb_x>0) {
                if ((fixed[mb_xy-1]==MV_CHANGED || fixed[mb_xy-1]==MV_FROZEN) && !IS_INTRA(s->current_picture.f.mb_type[mb_xy-1])) {
                    MV_index = mot_index - 4;
                    cluster[count][0] = mb_xy-1;
                    cluster[count][1] = s->current_picture.f.motion_val[0][MV_index][0];
                    cluster[count][2] = s->current_picture.f.motion_val[0][MV_index][1];
                    cluster[count][3] = s->current_picture.f.ref_index[0][4*cluster[count][0]];
                    sum_x+= (float)cluster[count][1];
                    sum_y+= (float)cluster[count][2];
                    sum_ref+= cluster[count][3];
                    sum_denom+= 1;
                    
                    count += 1;
                }
            }
            //Right estimate
            if (mb_x+1<s->mb_width) {
                if ((fixed[mb_xy+1]==MV_CHANGED || fixed[mb_xy+1]==MV_FROZEN) && !IS_INTRA(s->current_picture.f.mb_type[mb_xy+1])) {
                    MV_index = mot_index + 4;
                    cluster[count][0] = mb_xy+1;
                    cluster[count][1] = s->current_picture.f.motion_val[0][MV_index][0];
                    cluster[count][2] = s->current_picture.f.motion_val[0][MV_index][1];
                    cluster[count][3] = s->current_picture.f.ref_index[0][4*cluster[count][0]];
                    sum_x+= (float)cluster[count][1];
                    sum_y+= (float)cluster[count][2];
                    sum_ref+= cluster[count][3];
                    sum_denom+= 1;

                    count += 1;
                }
            }
            //Top estimate
            if (mb_y>0) {
                if ((fixed[mb_xy-s->mb_stride]==MV_CHANGED || fixed[mb_xy-s->mb_stride]==MV_FROZEN) && !IS_INTRA(s->current_picture.f.mb_type[mb_xy-s->mb_stride])) {
                    MV_index = mot_index - mot_stride*mot_step;
                    cluster[count][0] = mb_xy-s->mb_stride;
                    cluster[count][1] = s->current_picture.f.motion_val[0][MV_index][0];
                    cluster[count][2] = s->current_picture.f.motion_val[0][MV_index][1];
                    cluster[count][3] = s->current_picture.f.ref_index[0][4*cluster[count][0]];
                    sum_x+= (float)cluster[count][1];
                    sum_y+= (float)cluster[count][2];
                    sum_ref+= cluster[count][3];
                    sum_denom+= 1;
                    
                    count += 1;
                }
            }
            //Bottom estimate
            if (mb_y+1<s->mb_height) {
                if ((fixed[mb_xy+s->mb_stride]==MV_CHANGED || fixed[mb_xy+s->mb_stride]==MV_FROZEN) && !IS_INTRA(s->current_picture.f.mb_type[mb_xy+s->mb_stride])) {
                    MV_index = mot_index + mot_stride*mot_step;
                    cluster[count][0] = mb_xy+s->mb_stride;
                    cluster[count][1] = s->current_picture.f.motion_val[0][MV_index][0];
                    cluster[count][2] = s->current_picture.f.motion_val[0][MV_index][1];
                    cluster[count][3] = s->current_picture.f.ref_index[0][4*cluster[count][0]];
                    sum_x+= (float)cluster[count][1];
                    sum_y+= (float)cluster[count][2];
                    sum_ref+= cluster[count][3];
                    sum_denom+= 1;
                    
                    count += 1;
                }
            }
        } //Finish incorporating MV estimates
        #endif

		MVx_avg = (sum_x/sum_denom);
		MVy_avg = (sum_y/sum_denom);
		MV_ref_avg = floor((float)sum_ref/count + 0.5);
        
		*mv_func_x = (int)MVx_avg;
		*mv_func_y = (int)MVy_avg;
		*mv_func_ref = (int)MV_ref_avg;
		
    }
}
