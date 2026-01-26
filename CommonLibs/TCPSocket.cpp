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

#include "TCPSocket.h"
#include <cstring>
#include <fcntl.h>
#include <errno.h>
#include <iostream>

TCPClient::TCPClient(const std::string& server_ip, int server_port, int timeout_sec)
    : server_ip_(server_ip), server_port_(server_port), timeout_sec_(timeout_sec), sock_(-1) {
    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) {
        std::cout << "Failed to create socket" << std::endl;
        return;
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(server_port_);

    if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr_.sin_addr) <= 0) {
        std::cout << "Invalid IP address" << std::endl;
        return;
    }

    // Set receive timeout
    struct timeval timeout;
    timeout.tv_sec = timeout_sec_;
    timeout.tv_usec = 0;
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

TCPClient::~TCPClient() {
    if (sock_ >= 0) {
        close(sock_);
    }
}

bool TCPClient::connectToServer() {
    // Set socket to non-blocking
    int flags = fcntl(sock_, F_GETFL, 0);
    fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

    int result = connect(sock_, (struct sockaddr*)&server_addr_, sizeof(server_addr_));
    if (result < 0 && errno != EINPROGRESS) {
        std::cout << "SOCKET: Immediate connection failed " << result << ":" << errno << std::endl;
        return false;
    }
    std::cout << "SOCKET: Connected " << result << ":" << errno << std::endl;
    fd_set writefds;
    FD_ZERO(&writefds);
    FD_SET(sock_, &writefds);

    struct timeval timeout;
    timeout.tv_sec = timeout_sec_;
    timeout.tv_usec = 0;

    result = select(sock_ + 1, nullptr, &writefds, nullptr, &timeout);
    if (result <= 0) {
        return false;
    }

    int so_error = ENOTSOCK;
    socklen_t len = sizeof(so_error);
    getsockopt(sock_, SOL_SOCKET, SO_ERROR, &so_error, &len);
    if(so_error != 0)
    { 
        if (so_error == ETIMEDOUT) {
            std::cout << "Timeout Error = " << so_error << std::endl;
        }
        else {
            
            sleep(timeout_sec_);
            std::cout << "Error = " << so_error << " timeout = " << timeout_sec_ << std::endl;
        }
        return false;
    } 

    // Restore blocking mode
    fcntl(sock_, F_SETFL, flags);
    return true;
}

void TCPClient::sendData(const std::string& data) {
    if (send(sock_, data.c_str(), data.size(), 0) < 0) {
        std::cout << "Send failed" << std::endl;
    }
}
void TCPClient::sendBinaryData(const char *data, std::size_t len) {
    if (send(sock_, data, len, 0) < 0) {
        std::cout << "Send failed" << std::endl;
    }
}

std::string TCPClient::receiveData(size_t buffer_size) {
    uint8_t buffer[buffer_size];
    ssize_t bytes_received = recv(sock_, buffer, buffer_size, 0);
    //td::cout << "bytes_received = " << bytes_received  << std::endl;
    if (bytes_received < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            //std::cout << "OK: Receive timed out" << std::endl;
        } else {
            std::cout << "ERROR: Receive failed" << std::endl;
        }
    }    
    std::string str;    
    if (bytes_received > 0)
    {        
        str.append((char *)buffer,bytes_received);     
    }
    
    return str;
}
