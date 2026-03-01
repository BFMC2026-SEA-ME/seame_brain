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

import { Component, Input, ViewChild, ElementRef } from '@angular/core';
import { Subscription } from 'rxjs';
import { WebSocketService} from '../../webSocket/web-socket.service'

import { CommonModule } from '@angular/common';

import { MapSemaphoreComponent } from './map-semaphore/map-semaphore.component';
 
interface Semaphore { 
  x: number;
  y: number;
  state: string;
}

interface MapNode {
  id: string;
  x: number;
  y: number;
  xPct: number;
  yPct: number;
}

@Component({
  selector: 'app-map',
  standalone: true,
  imports: [MapSemaphoreComponent, CommonModule],
  templateUrl: './map.component.html',
  styleUrl: './map.component.css'
})
export class MapComponent {
  @Input() cursorRotation: number = 0;

  @ViewChild('imageElement') imageElementRef!: ElementRef<HTMLImageElement>;
  @ViewChild('imageContainer') imageContainerRef!: ElementRef<HTMLImageElement>;
  @ViewChild('overlayElement') overlayElementRef!: ElementRef<SVGElement>;

  private mapX: number = 0;
  private mapY: number = 0;

  private screenSize = {"width": 100, "height": 100}; // screen size in %
  private mapSize: number = 85; // map size in % for width
  private mapWidth: number = 0;
  private mapHeight: number = 0;

  private cursorSize: number = 6; // cursor size in % for width
  private semaphoreSize: number = 3;

  private semaphoreXOffset: number = 10;
  private semaphoreYOffset: number = 1.45;
  private hasLocation: boolean = false;
  
  public semaphores: Map<number, Semaphore> = new Map<number, Semaphore>();
  public graphNodes: MapNode[] = [];
  public pathPoints: string = '';
  public selectedNodeId: string | null = null;

  private graphBounds: { min_x: number; max_x: number; min_y: number; max_y: number } | null = null;
  private readonly mapWorldWidth = 20.67;
  private readonly mapWorldHeight = 13.76;

  private locationSubscription: Subscription | undefined;
  private semaphoresAndCarsSubscription: Subscription | undefined;
  private mapNodesSubscription: Subscription | undefined;
  private globalPathSubscription: Subscription | undefined;

  constructor( private  webSocketService: WebSocketService) { }
  
  ngOnInit()
  {
    this.locationSubscription = this.webSocketService.receiveLocation().subscribe(
      (message) => {
        this.hasLocation = true;
        this.mapX = (parseFloat(message.value.x)*100/20.67)
        this.mapY = (100 - parseFloat(message.value.y)*100/13.76) //magic percent + same system of coordinates
        this.updateMap()
      },
    );

    this.semaphoresAndCarsSubscription = this.webSocketService.receiveSemaphores().subscribe(
      (message) => {
        const recv = message.value;
        this.semaphores.set(recv.id, {x: recv.x, y: recv.y, state: recv.state});
      },
    );

    this.mapNodesSubscription = this.webSocketService.receiveMapNodes().subscribe(
      (message) => {
        const payload = (message as any)?.value ?? message;
        if (!payload || !payload.nodes) {
          return;
        }

        if (payload.bounds) {
          this.graphBounds = payload.bounds;
        }

        this.graphNodes = (payload.nodes as any[]).map((node) => {
          const world = this.graphToWorld(node.x, node.y);
          const pct = this.worldToPercent(world.x, world.y);
          return {
            id: String(node.id),
            x: world.x,
            y: world.y,
            xPct: pct.x,
            yPct: pct.y
          };
        });
        if (!this.hasLocation && this.graphBounds) {
          const centerGraphX = (this.graphBounds.min_x + this.graphBounds.max_x) / 2;
          const centerGraphY = (this.graphBounds.min_y + this.graphBounds.max_y) / 2;
          const centerWorld = this.graphToWorld(centerGraphX, centerGraphY);
          const centerPct = this.worldToPercent(centerWorld.x, centerWorld.y);
          this.mapX = centerPct.x;
          this.mapY = centerPct.y;
        }
        this.updateMap();
      },
    );

    this.globalPathSubscription = this.webSocketService.receiveGlobalPath().subscribe(
      (message) => {
        const payload = (message as any)?.value ?? message;
        const points = payload?.points as any[] | undefined;
        if (!points || points.length === 0) {
          this.pathPoints = '';
          return;
        }
        this.pathPoints = points.map((pt) => {
          const world = this.graphToWorld(pt.x, pt.y);
          const pct = this.worldToPercent(world.x, world.y);
          return `${pct.x},${pct.y}`;
        }).join(' ');
      },
    );
    this.webSocketService.sendMessageToFlask('{\"Name\": \"RequestMapNodes\", \"Value\": true}');
    this.updateMap()
  }

  ngOnDestroy() {
    if (this.locationSubscription) {
      this.locationSubscription.unsubscribe();
    }
    if (this.semaphoresAndCarsSubscription) {
      this.semaphoresAndCarsSubscription.unsubscribe();
    }
    if (this.mapNodesSubscription) {
      this.mapNodesSubscription.unsubscribe();
    }
    if (this.globalPathSubscription) {
      this.globalPathSubscription.unsubscribe();
    }
  }

  onLoadTrack(image: HTMLImageElement): void {
    const imageContainer = document.getElementById("map-track-image-container") as HTMLElement;

    if (imageContainer) {
      imageContainer.style.width = `${this.screenSize["width"]}%`;
      imageContainer.style.height = `${this.screenSize["height"]}%`;  
    }

    this.mapWidth = image.width;
    this.mapHeight = image.height;

    const map = document.getElementById("map-track-image") as HTMLElement;

    if (map) {
      map.style.width = `${this.mapSize}%`;
      map.style.height = `auto`;

      this.mapWidth = this.mapSize;
    }
  }

  onLoadCursor(): void {
    const cursor = document.getElementById("map-cursor") as HTMLElement;

    if (cursor) {
      cursor.style.width = `${this.cursorSize}%`;
      cursor.style.height = `auto`;
    }
  }

  onLoadSemaphore(id: number): void {
    const semaphore = document.getElementById("map-semaphore" + id) as HTMLElement;

    if (semaphore) {
      semaphore.style.position = "absolute";
      semaphore.style.width = `${this.semaphoreSize}%`;
      semaphore.style.height = `auto`;

      this.updateMap();
    }
  }

  updateMap(): void {
    const map = document.getElementById("map-track-image") as HTMLElement;
    const overlay = document.getElementById("map-track-overlay") as HTMLElement;
    let imageContainerHeight: number = 0;

    if (map) {
      if (this.imageContainerRef) {
        const imgContainer = this.imageContainerRef.nativeElement;
        const rect = imgContainer.getBoundingClientRect();
        imageContainerHeight = rect.height;
      }

      if (this.imageElementRef) {
        const image = this.imageElementRef.nativeElement;
        this.mapWidth = this.mapSize;
        this.mapHeight = (100 * image.height) / imageContainerHeight;
      }

      const top = (this.mapY * this.mapHeight) / 100 - this.mapHeight - (this.screenSize["height"] / 2 - this.mapHeight);
      const left = (this.mapX * this.mapWidth) / 100 - this.mapWidth - (this.screenSize["width"] / 2 - this.mapWidth);

      if (!this.hasLocation) {
        map.style.top = `0%`;
        map.style.left = `0%`;
        if (overlay) {
          overlay.style.top = `0%`;
          overlay.style.left = `0%`;
          overlay.style.width = `${this.mapSize}%`;
          overlay.style.height = `${this.mapHeight}%`;
        }
      } else {
        map.style.top = `${-top}%`;
        map.style.left = `${-left}%`;
        if (overlay) {
          overlay.style.top = `${-top}%`;
          overlay.style.left = `${-left}%`;
          overlay.style.width = `${this.mapSize}%`;
          overlay.style.height = `${this.mapHeight}%`;
        }
      }

      this.semaphores.forEach((value: Semaphore, key: number) => {
        const semaphore = document.getElementById("map-semaphore" + key) as HTMLElement;

        if (semaphore) { 
          const x = (value.x * 100/20.67);
          const y = (value.y * 100/13.76);

          const top_new = (y * this.mapHeight) / 100;
          const left_new = (x * this.mapWidth) / 100;
          
          semaphore.style.top = `${(-top - this.semaphoreXOffset) + top_new}%`;
          semaphore.style.left = `${(-left - this.semaphoreYOffset) + left_new}%`;
        }
      });
    }
  }

  onSelectNode(nodeId: string): void {
    this.selectedNodeId = nodeId;
    this.webSocketService.sendMessageToFlask(
      `{\"Name\": \"GlobalPlanningGoalNodeId\", \"Value\": \"${nodeId}\"}`
    );
  }

  private graphToWorld(x: number, y: number): { x: number; y: number } {
    if (!this.graphBounds) {
      return { x, y };
    }
    const spanX = Math.max(0.0001, this.graphBounds.max_x - this.graphBounds.min_x);
    const spanY = Math.max(0.0001, this.graphBounds.max_y - this.graphBounds.min_y);

    const scaleX = this.mapWorldWidth / spanX;
    const scaleY = this.mapWorldHeight / spanY;

    return {
      x: (x - this.graphBounds.min_x) * scaleX,
      y: (y - this.graphBounds.min_y) * scaleY
    };
  }

  private worldToPercent(x: number, y: number): { x: number; y: number } {
    return {
      x: (x * 100) / this.mapWorldWidth,
      y: 100 - (y * 100) / this.mapWorldHeight
    };
  }
}
