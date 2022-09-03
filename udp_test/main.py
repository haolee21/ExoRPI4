from UdpClient import *


def main():
    udp_client = UdpClient()
    udp_client.Connect()
    udp_client.ReqData()
    udp_client.CheckRecv()




if __name__ == "__main__":
    main()