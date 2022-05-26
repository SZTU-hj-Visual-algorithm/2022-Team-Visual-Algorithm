#pragma once
#include <Eigen/Dense>

class kal_filter
{
private:
	double dt;
	Eigen::Matrix<double, 2, 6> H;
	Eigen::Matrix<double, 6, 6> F;
	Eigen::Matrix<double, 2, 2> R;
	Eigen::Matrix<double, 6, 6> Q;
	Eigen::Matrix<double, 6, 6> sigma;
	
	Eigen::Matrix<double, 6, 1> Xk;
	Eigen::Matrix<double, 6, 2> K;
public:
	//��ʼ��
	Eigen::Matrix<double, 6, 1> Xk_1;
	kal_filter()
	{
		H << 1, 0, 0, 0, 0, 0,
				0, 0, 0, 1, 0, 0 ;
		R << 0.001, 0,
				0, 0.001;
		Q << 15, 0, 0, 0, 0, 0,
				0, 4, 0, 0, 0, 0,
				0, 0, 10, 0, 0, 0,
				0, 0, 0, 15, 0, 0,
				0, 0, 0, 0, 4, 0,
				0, 0, 0, 0, 0, 10;
		sigma = Q;
		Xk_1 << 0, 0.0001, 0.001, 0, 0.0000001, 0.000001;
	}
	void reset()
	{
		H << 1, 0, 0, 0, 0, 0,
				0, 0, 0, 1, 0, 0 ;
		R << 0.001, 0,
				0, 0.001;
		Q << 15, 0, 0, 0, 0, 0,
				0, 4, 0, 0, 0, 0,
				0, 0, 10, 0, 0, 0,
				0, 0, 0, 15, 0, 0,
				0, 0, 0, 0, 4, 0,
				0, 0, 0, 0, 0, 10;
		sigma = Q;
		Xk_1 << 0, 0.0001, 0.001, 0, 0.0000001, 0.000001;
	}
	
	
	//Ԥ�ⲽ������һʱ�̵ĺ������ͨ��Ԥ�ⷽ�̵õ���
	Eigen::Matrix<double, 6, 1> predict(double delata_t , bool predict)
	{
		dt = delata_t;
		F << 1, dt, dt* dt / 2, 0, 0, 0,
				0, 1, dt, 0, 0, 0,
				0, 0, 1, 0, 0, 0,
				0, 0, 0, 1, dt/6, dt* dt / 72,
				0, 0, 0, 0, 1, dt/6,
				0, 0, 0, 0, 0, 1;
		if (!predict)
		{
			Xk = F * Xk_1;
			return Xk;
		}
		else
		{
			return F * Xk_1;
		}
		
		
	}
	
	//���²�������һʱ�̵��������ֵ����Э������󡢿������������һʱ�̵ĺ������
	Eigen::Matrix<double, 6,1> correct(Eigen::Matrix<double,2,1> measured)
	{
		sigma = F * sigma * F.transpose() + Q;
		K = sigma * H.transpose() * (H * sigma * H.transpose() + R).inverse();
		Xk_1 = Xk + K * (measured - H * Xk);
		sigma = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * sigma;
		return Xk_1;
	}
	
};
