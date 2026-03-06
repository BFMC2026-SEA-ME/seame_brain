// Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//  1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import { CommonModule } from '@angular/common';
import { Component } from '@angular/core';
import { Subscription } from 'rxjs';
import { WebSocketService} from '../../webSocket/web-socket.service'

@Component({
  selector: 'app-live-camera',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './live-camera.component.html',
  styleUrl: './live-camera.component.css'
})
export class LiveCameraComponent {
  public image: string | undefined;
  public loading: boolean = true;
  private canvasSize: number[] = [512, 270];
  private cameraSubscription: Subscription | undefined;
  private loadingTimeout: any;
  private objectUrl: string | null = null;
  private blackImage: string = '';

  constructor( private  webSocketService: WebSocketService) { }

  ngOnInit()
  {  
    this.blackImage = this.createBlackImage();
    this.image = this.blackImage;

    this.cameraSubscription = this.webSocketService.receiveCamera().subscribe(
      (message) => {
        // 바이너리(ArrayBuffer/Blob/Buffer) 또는 base64 문자열 모두 처리
        const payload = (message as any)?.value ?? message;

        if (payload instanceof Blob) {
          this.setBlobImage(payload);
        } else if (payload instanceof ArrayBuffer) {
          const blob = new Blob([payload], { type: 'image/jpeg' });
          this.setBlobImage(blob);
        } else if (payload && (payload as any).type === 'Buffer' && Array.isArray((payload as any).data)) {
          const blob = new Blob([new Uint8Array((payload as any).data)], { type: 'image/jpeg' });
          this.setBlobImage(blob);
        } else if (typeof payload === 'string') {
          this.revokeObjectUrl();
          // 이미 data URL이면 그대로, 아니면 base64로 가정
          this.image = payload.startsWith('data:image')
            ? payload
            : `data:image/jpeg;base64,${payload}`;
        } else {
          // 알 수 없는 타입이면 블랙 이미지로 리셋
          this.setBlackImage();
        }
        this.loading = false;
        // Reset the loading timeout on each new image
        if (this.loadingTimeout) {
          clearTimeout(this.loadingTimeout);
        }
        this.loadingTimeout = setTimeout(() => {
          this.loading = true;
          this.setBlackImage();
        }, 2000);
      },
      (error) => {
        this.setBlackImage();
        this.loading = true;
        console.error('Error receiving disk usage:', error);
      }
    );
  }

  ngOnDestroy() {
    if (this.cameraSubscription) {
      this.cameraSubscription.unsubscribe();
    }
    if (this.loadingTimeout) {
      clearTimeout(this.loadingTimeout);
    }
    this.revokeObjectUrl();
  }

  private setBlobImage(blob: Blob): void {
    const nextUrl = URL.createObjectURL(blob);
    this.revokeObjectUrl();
    this.objectUrl = nextUrl;
    this.image = nextUrl;
  }

  private revokeObjectUrl(): void {
    if (!this.objectUrl) {
      return;
    }
    URL.revokeObjectURL(this.objectUrl);
    this.objectUrl = null;
  }

  private setBlackImage(): void {
    this.revokeObjectUrl();
    this.image = this.blackImage;
  }

  createBlackImage(): string {
    const canvas = document.createElement('canvas');
    canvas.width = this.canvasSize[0]; 
    canvas.height = this.canvasSize[1]; 
    const ctx = canvas.getContext('2d');
    if (ctx) {
      ctx.fillStyle = 'black'; 
      ctx.fillRect(0, 0, canvas.width, canvas.height);
    }
    return canvas.toDataURL('image/png'); 
  };
}
