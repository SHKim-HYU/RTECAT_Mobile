#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <malloc.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include "xddp_packet.h"

pthread_t nrt;


static void fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}

static void *regular_thread(void *arg)
{
	char *devname;
	int fd, ret;

	size_t BUFLEN = sizeof(packet::Odometry);
	printf("%d\n", BUFLEN);
	struct packet::Odometry *odom_msg = (packet::Odometry *)malloc(BUFLEN);

	if (asprintf(&devname, "/dev/rtp%d", XDDP_PORT_ODOM) < 0)
		fail("asprintf");

	fd = open(devname, O_RDONLY);
	free(devname);
	if (fd < 0)
		fail("open");

	for (;;) {
		/* Get the next message from realtime_thread. */
		ret = read(fd, (void *)odom_msg, BUFLEN);
		if (ret <= 0)
			fail("read");
		else{
			printf("ret: %d\n",ret);
			printf("Odometry msg\n\tPose\n");
			printf("\tx: %lf, y: %lf\n",odom_msg->pose.position.x, odom_msg->pose.position.y);
			printf("\tQuaternion\n");
			printf("\tx: %lf, y: %lf, z: %lf, w: %lf\n", odom_msg->pose.orientation.x, odom_msg->pose.orientation.y, odom_msg->pose.orientation.z, odom_msg->pose.orientation.w);
			printf("\tTwist\n");
			printf("\tx: %lf, y: %lf, z: %lf\n\n", odom_msg->twist.linear.x, odom_msg->twist.linear.y, odom_msg->twist.angular.z);
		}

	}

	return NULL;
}

int main(int argc, char **argv)
{
	pthread_attr_t regattr;
	sigset_t set;
	int sig;

	sigemptyset(&set);
	sigaddset(&set, SIGINT);
	sigaddset(&set, SIGTERM);
	sigaddset(&set, SIGHUP);
	pthread_sigmask(SIG_BLOCK, &set, NULL);

	pthread_attr_init(&regattr);
	pthread_attr_setdetachstate(&regattr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setinheritsched(&regattr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&regattr, SCHED_OTHER);

	errno = pthread_create(&nrt, &regattr, &regular_thread, NULL);
	if (errno)
		fail("pthread_create");

	sigwait(&set, &sig);
	pthread_cancel(nrt);
	pthread_join(nrt, NULL);

	return 0;
}
