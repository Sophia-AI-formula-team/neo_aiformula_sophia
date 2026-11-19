"""Loopback-published TCP relay. Only this sidecar joins a routable network."""
import select
import socket


def serve(client):
    with socket.create_connection(('driver', 16826), timeout=4) as driver:
        driver.settimeout(None)
        while True:
            ready, _, _ = select.select([client, driver], [], [])
            for source in ready:
                data = source.recv(65536)
                if not data:
                    return
                (driver if source is client else client).sendall(data)


with socket.socket() as listener:
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(('0.0.0.0', 16826))
    listener.listen(2)
    while True:
        conn, _ = listener.accept()
        with conn:
            try:
                serve(conn)
            except OSError as exc:
                print('Connection ended:', exc, flush=True)
