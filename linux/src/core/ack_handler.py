# core/ack_handler.py
"""
ACK/NACK handler for UAVX telemetry
"""

from PyQt5.QtCore import QTimer, QObject, pyqtSignal
from PyQt5.QtWidgets import QPushButton, QMenu


class AckHandler(QObject):
    """Handle ACK/NACK responses from FC with visual feedback"""
    
    ack_received = pyqtSignal(int, bool)
    nack_received = pyqtSignal(int)
    
    def __init__(self):
        super().__init__()
        self.pending_requests = {}
        self.debug_callback = None
        self._request_counter = 0
        self.timeout_timer = None
        self.timeout_duration = 10000  # 10 second timeout
    
    def init_timer(self):
        """Initialize timer in the main thread"""
        if self.timeout_timer is None:
            self.timeout_timer = QTimer()
            self.timeout_timer.timeout.connect(self.check_timeouts)
            self.timeout_timer.start(1000)
    
    def set_debug_callback(self, callback):
        """Set a callback for debug messages"""
        self.debug_callback = callback
    
    def _debug(self, msg):
        """Send debug message if callback is set"""
        if self.debug_callback:
            self.debug_callback(msg)
        else:
            print(f"[ACK] {msg}")
    
    def register_button(self, tag: int, button: QPushButton, callback=None):
        """Register a button for ACK/NACK feedback - returns request_id"""
        self._debug(f"Registering button for tag {tag}")
        self.init_timer()
        self._request_counter += 1
        request_id = self._request_counter
        self._debug(f"  Generated request_id={request_id}")
        self.pending_requests[request_id] = {
            'tag': tag,
            'button': button,
            'callback': callback,
            'started': False,
            'timestamp': QTimer(),
            'timeout_count': 0
        }
        button.setProperty('request_id', request_id)
        if not button.property('original_text'):
            button.setProperty('original_text', button.text())
        self._debug(f"  Button registered with request_id={request_id}")
        return request_id
    
    def handle_ack(self, tag: int, success: bool):
        """
        Handle ACK/NACK response - finds the request by tag.
        
        In UAVX protocol:
        - success = True means ACK (Reason != 0, typically 255)
        - success = False means NACK (Reason == 0)
        """
        self._debug(f"🔔 handle_ack called: tag={tag}, success={success}")
        
        # Ignore AFName ACKs (tag 223 and 241 are AFName packet related)
        # These are sent automatically by the FC and we don't need to handle them
        if tag == 223 or tag == 241:
            self._debug(f"  ⏭️ Ignoring AFName ACK for tag {tag}")
            return
        
        found_id = None
        for req_id, req in self.pending_requests.items():
            if req['tag'] == tag:
                found_id = req_id
                break
        
        if found_id is not None:
            self._debug(f"  ✅ Found pending request with ID {found_id}")
            req = self.pending_requests[found_id]
            button = req['button']
            callback = req['callback']
            
            if success:
                self._set_button_state(button, 'success')
                if callback:
                    callback()
                self.ack_received.emit(tag, True)
            else:
                self._set_button_state(button, 'failed')
                self.nack_received.emit(tag)
            
            # Reset after 2 seconds
            QTimer.singleShot(2000, lambda: self._reset_button(button, req))
            del self.pending_requests[found_id]
        else:
            self._debug(f"  ❌ No pending request for tag {tag}")
    
    def check_timeouts(self):
        """Check for timed out requests"""
        timeout_ids = []
        for req_id, req in self.pending_requests.items():
            if req['started'] and req['timestamp'].remainingTime() < 0:
                req['timeout_count'] += 1
                if req['timeout_count'] >= 1:
                    self._debug(f"⏱️ Timeout for request_id {req_id} (tag={req['tag']})")
                    button = req['button']
                    self._set_button_state(button, 'timeout')
                    timeout_ids.append(req_id)
        
        for req_id in timeout_ids:
            del self.pending_requests[req_id]
    
    def request_sent(self, request_id: int):
        """Called when a request is sent - sets button to waiting state"""
        self._debug(f"Request sent for request_id={request_id}")
        if request_id in self.pending_requests:
            req = self.pending_requests[request_id]
            req['timestamp'].start(self.timeout_duration)
            req['started'] = True
            req['timeout_count'] = 0
            self._set_button_state(req['button'], 'waiting')
        else:
            self._debug(f"  ❌ No pending request for request_id={request_id}")
    
    def reset_button(self, request_id: int):
        """Manually reset a button (called by user)"""
        self._debug(f"🔄 Manual reset for request_id={request_id}")
        if request_id in self.pending_requests:
            req = self.pending_requests[request_id]
            button = req['button']
            self._reset_button(button, req)
            del self.pending_requests[request_id]
        else:
            # Try to find by tag
            for req_id, req in list(self.pending_requests.items()):
                if req['button'].property('request_id') == request_id:
                    self._reset_button(req['button'], req)
                    del self.pending_requests[req_id]
                    break
    
    def _reset_button(self, button: QPushButton, req: dict):
        """Reset button to ready state"""
        self._debug(f"🔄 Resetting button")
        button.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                font-weight: bold;
                border: 2px solid #229954;
                border-radius: 4px;
                padding: 4px 8px;
            }
            QPushButton:hover {
                background-color: #229954;
            }
        """)
        button.setEnabled(True)
        original = button.property('original_text')
        if original:
            button.setText(original)
        button.setProperty('request_id', None)
        self._debug(f"  Button reset - ready for next request")
    
    def _set_button_state(self, button: QPushButton, state: str):
        """Set button visual state"""
        styles = {
            'waiting': """
                QPushButton {
                    background-color: #f39c12;
                    color: white;
                    font-weight: bold;
                    border: 2px solid #e67e22;
                    border-radius: 4px;
                    padding: 4px 8px;
                }
            """,
            'success': """
                QPushButton {
                    background-color: #27ae60;
                    color: white;
                    font-weight: bold;
                    border: 2px solid #229954;
                    border-radius: 4px;
                    padding: 4px 8px;
                }
            """,
            'failed': """
                QPushButton {
                    background-color: #e74c3c;
                    color: white;
                    font-weight: bold;
                    border: 2px solid #c0392b;
                    border-radius: 4px;
                    padding: 4px 8px;
                }
            """,
            'timeout': """
                QPushButton {
                    background-color: #95a5a6;
                    color: white;
                    font-weight: bold;
                    border: 2px solid #7f8c8d;
                    border-radius: 4px;
                    padding: 4px 8px;
                }
            """
        }
        
        if state in styles:
            button.setStyleSheet(styles[state])
        else:
            button.setStyleSheet("""
                QPushButton {
                    background-color: #27ae60;
                    color: white;
                    font-weight: bold;
                    border: 2px solid #229954;
                    border-radius: 4px;
                    padding: 4px 8px;
                }
                QPushButton:hover {
                    background-color: #229954;
                }
            """)
        
        texts = {
            'waiting': '⏳ Waiting...',
            'success': '✅ Success',
            'failed': '❌ Failed',
            'timeout': '⏱️ Timeout'
        }
        
        if state in texts:
            button.setText(texts[state])
            if state == 'timeout':
                button.setEnabled(True)
            else:
                button.setEnabled(False)
        else:
            button.setEnabled(True)
            original = button.property('original_text')
            if original:
                button.setText(original)


# Global instance
ack_handler = AckHandler()
