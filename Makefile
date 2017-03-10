#
# Makefile for TinyEKF GPS example
#
# Copyright (C) 2015 Simon D. Levy
#
# This code is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This code is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this code.  If not, see <http:#www.gnu.org/licenses/>.

all: EKFTest

run: EKFTest
	./EKFTest

gps_ekf: EKFTest.cpp KalmanFilter.cpp KalmanFilter.hpp MatOp.hpp
	gcc -g -Wall -I. -I.. -o gps_ekf EKFTest.cpp KalmanFilter.cpp -lm

edit:
	vim EKFTest.cpp

clean:
	rm -f EKFTest *.o *~ ekf.csv

commit:
	git commit -a --allow-empty-message -m ''
	git push
