/*
 * Copyright 2026 Late Beam
 *
 * This software is distributed under the terms of the GNU General Public
 * License version 3. See the COPYING and NOTICE files in the current
 * directory for licensing information.
 *
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 */

#ifndef TCPCLIENT_HPP
#define TCPCLIENT_HPP

#include <string>
#include <stdexcept>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>

class TCPClient {
public:
    TCPClient(const std::string& server_ip, int server_port, int timeout_sec = 5);
    ~TCPClient();

    bool connectToServer();   

    void sendData(const std::string& data);
    void sendBinaryData(const char *data, std::size_t len);
    std::string receiveData(size_t buffer_size = 1024);

private:
    std::string server_ip_;
    int server_port_;
    int sock_;
    int timeout_sec_;
    struct sockaddr_in server_addr_;
};

#endif // TCPCLIENT_HPP
