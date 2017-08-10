#include "matrix3f.h"
#include "sys.h"

void matrix3f_copy(matrix3f_t *sou,matrix3f_t *dst)
{
	dst->a.x = sou->a.x;
	dst->a.y = sou->a.y;
	dst->a.z = sou->a.z;
	
	dst->b.x = sou->b.x;
	dst->b.y = sou->b.y;
	dst->b.z = sou->b.z;
	
	dst->c.x = sou->c.x;
	dst->c.y = sou->c.y;
	dst->c.z = sou->c.z;
}

//!
void matrix3f_rotate(matrix3f_t *mat,vector3f_t *g)
{
	matrix3f_t temp_matrix;
	temp_matrix.a.x = mat->a.y * g->z - mat->a.z * g->y;
	temp_matrix.a.y = mat->a.z * g->x - mat->a.x * g->z;
	temp_matrix.a.z = mat->a.x * g->y - mat->a.y * g->x;
	
	temp_matrix.b.x = mat->b.y * g->z - mat->b.z * g->y;
	temp_matrix.b.y = mat->b.z * g->x - mat->b.x * g->z;
	temp_matrix.b.z = mat->b.x * g->y - mat->b.y * g->x;
	
	temp_matrix.c.x = mat->c.y * g->z - mat->c.z * g->y;
	temp_matrix.c.y = mat->c.z * g->x - mat->c.x * g->z;
	temp_matrix.c.z = mat->c.x * g->y - mat->c.y * g->x;
	
	matrix3f_add(mat,&temp_matrix,NULL);
}


//只旋转X，Y轴
void matrix3f_rotateXYinv(matrix3f_t *mat,vector3f_t *g)
{
	matrix3f_t temp_matrix;
	temp_matrix.a.x =   mat->a.z * g->y;
	temp_matrix.a.y = - mat->a.z * g->x;
	temp_matrix.a.z = - mat->a.x * g->y + mat->a.y * g->x;
	
	temp_matrix.b.x =   mat->b.z * g->y;
	temp_matrix.b.y = - mat->b.z * g->x;
	temp_matrix.b.z = - mat->b.x * g->y + mat->b.y * g->x;
	
	temp_matrix.c.x =   mat->c.z * g->y;
	temp_matrix.c.y = - mat->c.z * g->x;
	temp_matrix.c.z = - mat->c.x * g->y + mat->c.y * g->x;
	
	matrix3f_add(mat,&temp_matrix,NULL);
}





void matrix3f_transport(matrix3f_t *mat,matrix3f_t *out)
{
	matrix3f_t mat_temp;
	if(out == NULL || out == mat)
	{
		matrix3f_copy(mat,&mat_temp);		
		out = mat;
		mat = &mat_temp;	
	}	
	
	out->a.x = mat->a.x;	
	out->a.y = mat->b.x;
	out->a.z = mat->c.x;
	
	out->b.x = mat->a.y;
	out->b.y = mat->b.y;
	out->b.z = mat->c.y;
	
	out->c.x = mat->a.z;
	out->c.y = mat->b.z;
	out->c.z = mat->c.z;
}


//!
void matrix3f_add(matrix3f_t *dst,matrix3f_t *sou,matrix3f_t *out)
{
	if(out == NULL)out = dst;
	
	out->a.x = dst->a.x + sou->a.x;
	out->a.y = dst->a.y + sou->a.y;
	out->a.z = dst->a.z + sou->a.z;

	out->b.x = dst->b.x + sou->b.x;
	out->b.y = dst->b.y + sou->b.y;
	out->b.z = dst->b.z + sou->b.z;
	
	out->c.x = dst->c.x + sou->c.x;
	out->c.y = dst->c.y + sou->c.y;
	out->c.z = dst->c.z + sou->c.z;
}


void matrix3f_minus(matrix3f_t *dst,matrix3f_t *sou,matrix3f_t *out)
{
	if(out == NULL)out = dst;

	out->a.x = dst->a.x - sou->a.x;
	out->a.y = dst->a.y - sou->a.y;
	out->a.z = dst->a.z - sou->a.z;

	out->b.x = dst->b.x - sou->b.x;
	out->b.y = dst->b.y - sou->b.y;
	out->b.z = dst->b.z - sou->b.z;
	
	out->c.x = dst->c.x - sou->c.x;
	out->c.y = dst->c.y - sou->c.y;
	out->c.z = dst->c.z - sou->c.z;
}


void matrix3f_mul(matrix3f_t *mat,vector3f_t *vec,vector3f_t *out)
{
	vector3f_t vec_temp;
	if(out == NULL || out == vec)
	{
		vector3f_copy(vec,&vec_temp);		
		out = vec;
		vec = &vec_temp;	
	}
	out->x = mat->a.x * vec->x + mat->a.y * vec->y + mat->a.z * vec->z;
	out->y = mat->b.x * vec->x + mat->b.y * vec->y + mat->b.z * vec->z;
	out->z = mat->c.x * vec->x + mat->c.y * vec->y + mat->c.z * vec->z;

}


//矩阵转置之后再相乘
void matrix3f_mul_transpose(matrix3f_t *mat,vector3f_t *vec,vector3f_t *out)
{
	vector3f_t vec_temp;
	if(out == NULL || out == vec)
	{
		vector3f_copy(vec,&vec_temp);		
		out = vec;
		vec = &vec_temp;	
	}
	out->x = mat->a.x * vec->x + mat->b.x * vec->y + mat->c.x * vec->z;
	out->y = mat->a.y * vec->x + mat->b.y * vec->y + mat->c.y * vec->z;
	out->z = mat->a.z * vec->x + mat->b.z * vec->y + mat->c.z * vec->z;
}

//只获取 x,y部分
void matrix3f_mulXY(matrix3f_t *mat, vector3f_t *vec, vector2f_t *out)
{
	out->x = mat->a.x * vec->x + mat->a.y * vec->y + mat->a.z * vec->z;
	out->y = mat->b.x * vec->x + mat->b.y * vec->y + mat->b.z * vec->z;

}

uint8_t matrix3f_isnan(matrix3f_t *mat)
{
	return (vector3f_isnan(&(mat->a)) || vector3f_isnan(&(mat->b)) || vector3f_isnan(&(mat->c)));
}


uint8_t matrix3f_iszero(matrix3f_t *mat)
{
	return (vector3f_iszero(&(mat->a)) && vector3f_iszero(&(mat->b)) && vector3f_iszero(&(mat->c)));
}


void matrix3f_zero(matrix3f_t *mat)
{
	mat->a.x = 0;
	mat->a.y = 0;
	mat->a.z = 0;
	
	mat->b.x = 0;
	mat->b.y = 0;
	mat->b.z = 0;
	
	mat->c.x = 0;
	mat->c.y = 0;
	mat->c.z = 0;
}


void matrix3f_identify(matrix3f_t *mat)
{
	mat->a.x = 1;
	mat->a.y = 0;
	mat->a.z = 0;
	
	mat->b.x = 0;
	mat->b.y = 1;
	mat->b.z = 0;
	
	mat->c.x = 0;
	mat->c.y = 0;
	mat->c.z = 1;
}



void matrix3f_to_euler(matrix3f_t *mat,float *roll,float *pitch,float *yaw)
{
    if (pitch != NULL) 
		{
        *pitch = -safe_asin(mat->c.x);
    }
		
    if (roll != NULL) 
		{
        *roll = atan2f(mat->c.y, mat->c.z);
    }
		
    if (yaw != NULL) 
		{
        *yaw = atan2f(mat->b.x, mat->a.x);
    }
}



void matrix3f_from_euler(matrix3f_t *mat,float roll,float pitch,float yaw)
{
	float cp = cosf(pitch);
	float sp = sinf(pitch);
	float sr = sinf(roll);
	float cr = cosf(roll);
	float sy = sinf(yaw);
	float cy = cosf(yaw);

	mat->a.x = cp * cy;
	mat->a.y = (sr * sp * cy) - (cr * sy);
	mat->a.z = (cr * sp * cy) + (sr * sy);
	
	mat->b.x = cp * sy;
	mat->b.y = (sr * sp * sy) + (cr * cy);
	mat->b.z = (cr * sp * sy) - (sr * cy);
	
	mat->b.x = -sp;
	mat->b.y = sr * cp;
	mat->b.z = cr * cp;
}

void matrix3f_from_quat(matrix3f_t *mat, float q0, float q1, float q2, float q3)
{
	//第一行
	mat->a.x = 1.0f - 2.0f*(q2*q2 + q3*q3);
	mat->a.y = 2.0f*(q1*q2 - q0*q3);
	mat->a.z = 2.0f*(q1*q3 + q0*q2);
	//第二行
	mat->b.x = 2.0f*(q1*q2 + q0*q3);
	mat->b.y = 1.0f - 2.0f*(q1*q1 + q3*q3);
	mat->b.z = 2.0f*(q2*q3 - q0*q1);
	//第三行
	mat->c.x = 2.0f*(q1*q3 - q0*q2);
	mat->c.y = 2.0f*(q2*q3 + q0*q1);
	mat->c.z = 1.0f - 2.0f*(q1*q1 + q2*q2);
}
void matrix3f_to_quat(matrix3f_t *mat, float *q0, float *q1, float *q2, float *q3)
{
	float temp;
	*q0 = 0.5f*sqrtf(1.0f + mat->a.x + mat->b.y + mat->c.z);
	temp = 0.25f / *q0;
	*q1 = temp*(mat->c.y - mat->b.z);
	*q2 = temp*(mat->a.z - mat->c.x);
	*q3 = temp*(mat->b.x - mat->a.y);

}

