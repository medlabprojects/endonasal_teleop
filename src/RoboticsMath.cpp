# pragma once
#include "RoboticsMath.h"

double RoboticsMath::deg2rad(double degrees) 
{
	return degrees * 4.0 * atan(1.0) / 180.0;
}

double RoboticsMath::sgn(double x)
{
	double s = (x > 0) - (x < 0);
	return s;
}

double RoboticsMath::vectornorm(Eigen::Vector3d v)
{
	return sqrt(v.transpose()*v);
}

Eigen::Matrix3d RoboticsMath::orthonormalize(Eigen::Matrix3d R)
{
	Eigen::Matrix3d R_ortho;
	R_ortho.fill(0);
	// Normalize the first column:
	R_ortho.col(0) = R.col(0) / vectornorm(R.col(0));

	// Orthogonalize & normalize second column:
	R_ortho.col(1) = R.col(1);
	double c = (R_ortho.col(1).transpose()*R_ortho.col(0));
	c = c / (R_ortho.col(0).transpose()*R_ortho.col(0));
	R_ortho.col(1) = R_ortho.col(1) - c*R_ortho.col(0);
	R_ortho.col(1) = R_ortho.col(1) / vectornorm(R_ortho.col(1));

	// Orthogonalize & normalize third column:
	R_ortho.col(2) = R.col(2);
	double d = (R_ortho.col(2).transpose()*R_ortho.col(0));
	d = d / (R_ortho.col(0).transpose()*R_ortho.col(0));
	R_ortho.col(2) = R_ortho.col(2) - d*R_ortho.col(0);
	double e = (R_ortho.col(2).transpose()*R_ortho.col(1));
	e = e / (R_ortho.col(1).transpose()*R_ortho.col(1));
	R_ortho.col(2) = R_ortho.col(2) - e*R_ortho.col(1);
	R_ortho.col(2) = R_ortho.col(2) / vectornorm(R_ortho.col(2));
	return R_ortho;
}

Eigen::Matrix4d RoboticsMath::assembleTransformation(Eigen::Matrix3d Rot, Eigen::Vector3d Trans)
{
	Rot = orthonormalize(Rot);
	Eigen::Matrix4d T;
	T.fill(0);
	T.topLeftCorner(3, 3) = Rot;
	T.topRightCorner(3, 1) = Trans;
	T(3, 3) = 1;
	return T;
}

RoboticsMath::Vector7d RoboticsMath::collapseTransform(Eigen::Matrix4d T)
{
	Eigen::Matrix<double, 7, 1> x;
	x.fill(0);
	x.head(3) = T.topRightCorner(3, 1);
	x.tail(4) = rotm2quat(T.topLeftCorner(3, 3));
	return x;
}

Eigen::Matrix3d RoboticsMath::quat2rotm(Eigen::Vector4d Quat)
{
	// Assuming convention q = [w x y z] (agrees w/Matlab)
	Eigen::Matrix3d R;
	R.fill(0);

	R(0, 0) = pow(Quat(0), 2) + pow(Quat(1), 2) - pow(Quat(2), 2) - pow(Quat(3), 2);
	R(0, 1) = 2 * Quat(1)*Quat(2) - 2 * Quat(0)*Quat(3);
	R(0, 2) = 2 * Quat(1)*Quat(3) + 2 * Quat(0)*Quat(2);

	R(1, 0) = 2 * Quat(1)*Quat(2) + 2 * Quat(0)*Quat(3);
	R(1, 1) = pow(Quat(0), 2) - pow(Quat(1), 2) + pow(Quat(2), 2) - pow(Quat(3), 2);
	R(1, 2) = 2 * Quat(2)*Quat(3) - 2 * Quat(0)*Quat(1);

	R(2, 0) = 2 * Quat(1)*Quat(3) - 2 * Quat(0)*Quat(2);
	R(2, 1) = 2 * Quat(2)*Quat(3) + 2 * Quat(0)*Quat(1);
	R(2, 2) = pow(Quat(0), 2) - pow(Quat(1), 2) - pow(Quat(2), 2) + pow(Quat(3), 2);
	return R;
}

Eigen::Vector4d RoboticsMath::rotm2quat(Eigen::Matrix3d R)
{
	R = orthonormalize(R);
	Eigen::Vector4d Q;
	Q.fill(0);

	double trace = R(0, 0) + R(1, 1) + R(2, 2);
	if (trace > 0)
	{
		double s = 0.5*sqrt(trace + 1.0);
		Q(0) = s; //w eqn
		Q(1) = (R(2, 1) - R(1, 2)) / (4 * s); //x eqn
		Q(2) = (R(0, 2) - R(2, 0)) / (4 * s); //y eqn
		Q(3) = (R(1, 0) - R(0, 1)) / (4 * s); //z eqn
	}
	else
	{
		if (R(0, 0)>R(1, 1) && R(0, 0)>R(2, 2))
		{
			double s = 0.5*sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
			Q(0) = (R(2, 1) - R(1, 2))*s; //w eqn
			Q(1) = s; //x eqn
			Q(2) = (R(0, 1) + R(1, 0))*s; //y eqn
			Q(3) = (R(0, 2) + R(2, 0))*s; //z eqn
		}
		else if (R(1, 1)>R(2, 2))
		{
			double s = 0.5*sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
			Q(0) = (R(0, 2) - R(2, 0))*s; //w eqn
			Q(1) = (R(0, 1) + R(1, 0))*s; //x eqn
			Q(2) = s; //y eqn
			Q(3) = (R(1, 2) + R(2, 1))*s; //z eqn
		}
		else
		{
			double s = 0.5*sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
			Q(0) = (R(1, 0) - R(0, 1))*s; //w eqn
			Q(1) = (R(0, 2) + R(2, 0))*s; //x eqn
			Q(2) = (R(1, 2) + R(2, 1))*s; //y eqn
			Q(3) = s; //z eqn
		}
	}

	return Q;
}

void RoboticsMath::getCofactor(double A[6][6], double temp[6][6], int p, int q, int n) // TODO: is this used? useful? no return?
{
	// Function to get cofactor of A[p][q] in temp[][]
	int i = 0, j = 0;

	// Looping for each element of the matrix
	for (int row = 0; row < n; row++)
	{
		for (int col = 0; col < n; col++)
		{
			//  Copying into temporary matrix only those element
			//  which are not in given row and column
			if (row != p && col != q)
			{
				temp[i][j++] = A[row][col];

				// Row is filled, so increase row index and
				// reset col index
				if (j == n - 1)
				{
					j = 0;
					i++;
				}
			}
		}
	}
}

double RoboticsMath::determinant(double A[6][6], int n)
{
	// Recursive function for finding determinant of matrix.
	double D = 0; // Initialize result

				  //  Base case : if matrix contains single element
	if (n == 1)
		return A[0][0];

	double temp[6][6]; // To store cofactors

	int sign = 1;  // To store sign multiplier

				   // Iterate for each element of first row
	for (int f = 0; f < n; f++)
	{
		// Getting Cofactor of A[0][f]
		getCofactor(A, temp, 0, f, n);
		D += sign * A[0][f] * determinant(temp, n - 1);

		// terms are to be added with alternate sign
		sign = -sign;
	}

	return D;
}

void RoboticsMath::adjoint(double A[6][6], double adj[6][6])
{
	// Function to get adjoint of A[N][N] in adj[N][N].

	// temp is used to store cofactors of A[][]
	int sign = 1;
	double temp[6][6];

	for (int i = 0; i<6; i++)
	{
		for (int j = 0; j<6; j++)
		{
			// Get cofactor of A[i][j]
			getCofactor(A, temp, i, j, 6);

			// sign of adj[j][i] positive if sum of row
			// and column indexes is even.
			sign = ((i + j) % 2 == 0) ? 1 : -1;

			// Interchanging rows and columns to get the
			// transpose of the cofactor matrix
			adj[j][i] = (sign)*(determinant(temp, 6 - 1));
		}
	}
}

void RoboticsMath::inverse(double A[6][6], double inverse[6][6]) // TODO: for loops using best practices?
{
	// Function to calculate and store inverse, returns false if
	// matrix is singular

	// Find determinant of A[][]
	double det = determinant(A, 6);
	if (det != 0) //matrix is not singular
	{
		// Find adjoint
		double adj[6][6];
		adjoint(A, adj);

		// Find Inverse using formula "inverse(A) = adj(A)/det(A)"
		for (int i = 0; i<6; i++)
			for (int j = 0; j<6; j++)
				inverse[i][j] = adj[i][j] / double(det);
	}
}

Eigen::Matrix3d RoboticsMath::hat3(Eigen::Vector3d v)
{
	Eigen::Matrix3d H = Eigen::Matrix<double, 3, 3>::Zero();
	H(0, 1) = -1 * v(2);
	H(0, 2) = v(1);
	H(1, 0) = v(2);
	H(1, 2) = -1 * v(0);
	H(2, 0) = -1 * v(1);
	H(2, 1) = v(0);

	return H;
}

RoboticsMath::Matrix6d RoboticsMath::Adjoint_p_q(Eigen::Vector3d p, Eigen::Vector4d q)
{
	Eigen::Matrix3d R = quat2rotm(q);
	Eigen::Matrix3d phat = hat3(p);
	Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Zero();
	Ad.topLeftCorner<3, 3>() = R;
	Ad.bottomRightCorner<3, 3>() = R;
	Ad.topRightCorner<3, 3>() = phat*R;

	return Ad;
}

Eigen::Matrix4d RoboticsMath::inverseTransform(Eigen::Matrix4d T)
{
	Eigen::Matrix4d Tinv;
	Tinv.fill(0);
	Tinv.topLeftCorner(3, 3) = T.topLeftCorner(3, 3).transpose();
	Tinv.topRightCorner(3, 1) = -1 * T.topLeftCorner(3, 3).transpose()*T.topRightCorner(3, 1);
	Tinv(3, 3) = 1.0;
	return Tinv;
}

Eigen::Vector4d RoboticsMath::slerp(Eigen::Vector4d qa, Eigen::Vector4d qb, double t)
{
	Eigen::Vector4d qm;
	qm.fill(0);

	double cosHalfTheta = qa.transpose()*qb;
	if (abs(cosHalfTheta) >= 1.0)
	{
		qm = qa;
		return qm;
	}

	double halfTheta = acos(cosHalfTheta);
	double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);

	if (abs(sinHalfTheta)<0.001)
	{
		qm(0) = 0.5*qa(0) + 0.5*qb(0);
		qm(1) = 0.5*qa(1) + 0.5*qb(1);
		qm(2) = 0.5*qa(2) + 0.5*qb(2);
		qm(3) = 0.5*qa(3) + 0.5*qb(3);
		return qm;
	}

	double ratioA = sin((1 - t)*halfTheta) / sinHalfTheta;
	double ratioB = sin(t*halfTheta) / sinHalfTheta;

	qm(0) = ratioA*qa(0) + ratioB*qb(0);
	qm(1) = ratioA*qa(1) + ratioB*qb(1);
	qm(2) = ratioA*qa(2) + ratioB*qb(2);
	qm(3) = ratioA*qa(3) + ratioB*qb(3);
	return qm;
}

Eigen::Matrix<double, 4, Eigen::Dynamic> RoboticsMath::quatInterp(Eigen::Matrix<double,4,Eigen::Dynamic> refQuat, Eigen::VectorXd refArcLengths, Eigen::VectorXd interpArcLengths)
{
	int count = 0;
	int N = interpArcLengths.size();
	Eigen::MatrixXd quatInterpolated(4, N);
	quatInterpolated.fill(0);

	for (int i = 0; i<N; i += 1)
	{
		if (interpArcLengths(i) < refArcLengths(count + 1)) {
			count = count + 1;
		}
		double L = refArcLengths(count) - refArcLengths(count + 1);
		double t = (refArcLengths(count) - interpArcLengths(i)) / L;
		quatInterpolated.col(i) = slerp(refQuat.col(count), refQuat.col(count + 1), t);
	}

	return quatInterpolated;
}
