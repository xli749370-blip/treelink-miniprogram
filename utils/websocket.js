// 封装微信 WebSocket：连接 ESP32 的 WebSocket 服务器
// 注意：连本地 LAN 设备需在 DevTools 勾选「不校验合法域名」

class DroneSocket {
  constructor() {
    this.task = null;
    this.connected = false;
  }

  connect(ip, port, handlers) {
    this.close();
    const url = `ws://${ip}:${port}`;
    this.task = wx.connectSocket({ url });

    this.task.onOpen(() => {
      this.connected = true;
      if (handlers && handlers.onOpen) handlers.onOpen();
    });
    this.task.onClose(() => {
      this.connected = false;
      if (handlers && handlers.onClose) handlers.onClose();
    });
    this.task.onError((err) => {
      if (handlers && handlers.onError) handlers.onError(err);
    });
    this.task.onMessage((res) => {
      if (handlers && handlers.onMessage) handlers.onMessage(res);
    });
  }

  send(buf) {
    if (this.task && this.connected) {
      this.task.send({ data: buf });
    }
  }

  close() {
    if (this.task) {
      try { this.task.close({}); } catch (e) {}
      this.task = null;
    }
    this.connected = false;
  }
}

module.exports = DroneSocket;
