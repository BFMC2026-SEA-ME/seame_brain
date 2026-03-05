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
  xSvg: number;
  ySvg: number;
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

  @ViewChild('imageContainer') imageContainerRef!: ElementRef<HTMLImageElement>;
  @ViewChild('overlayElement') overlayElementRef!: ElementRef<SVGElement>;

  private mapX: number = 0;
  private mapY: number = 0;
  private enableMapPan: boolean = false;
  private readonly mapImageWidth = 772;
  private readonly mapImageHeight = 600;
  private readonly mapImageBounds = {
    minX: 28,
    minY: 18,
    maxX: 732,
    maxY: 564
  };
  private readonly mapFitPaddingRatio = 0.06;

  private screenSize = {"width": 100, "height": 100}; // screen size in %
  private mapSize: number = 50; // map size in % for width
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
  public currentPoseSvg: { x: number; y: number } | null = null;
  public currentPoseNodeId: string | null = null;
  public checkpointNodeIds: Set<string> = new Set([
    '11', '25', '33', '39', '46', '60', '73', '76',
    '156', '103', '130', '117', '140', '90', '81', '150'
  ]);
  public passedCheckpointNodeIds: Set<string> = new Set<string>();

  private graphBounds: { min_x: number; max_x: number; min_y: number; max_y: number } | null = null;
  private currentPoseGraph: { x: number; y: number } | null = null;

  private locationSubscription: Subscription | undefined;
  private semaphoresAndCarsSubscription: Subscription | undefined;
  private mapNodesSubscription: Subscription | undefined;

  constructor( private  webSocketService: WebSocketService) { }
  
  ngOnInit()
  {
    this.locationSubscription = this.webSocketService.receiveGlobalPose().subscribe(
      (message) => {
        const payload = (message as any)?.value ?? message;
        if (!payload) {
          return;
        }
        const locX = Number(payload.x);
        const locY = Number(payload.y);
        if (!Number.isFinite(locX) || !Number.isFinite(locY)) {
          return;
        }

        this.hasLocation = true;
        this.currentPoseGraph = { x: locX, y: locY };
        this.currentPoseSvg = this.graphToSvg(locX, locY);
        this.currentPoseNodeId = this.findNearestNodeId(locX, locY);
        this.markCheckpointAsPassed(this.currentPoseNodeId);
        this.updateMap();
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
          const svg = this.graphToSvg(node.x, node.y);
          return {
            id: String(node.id),
            x: Number(node.x),
            y: Number(node.y),
            xSvg: svg.x,
            ySvg: svg.y
          };
        });
        if (!this.hasLocation && this.graphBounds) {
          const centerGraphX = (this.graphBounds.min_x + this.graphBounds.max_x) / 2;
          const centerGraphY = (this.graphBounds.min_y + this.graphBounds.max_y) / 2;
          const centerPct = this.graphToPercent(centerGraphX, centerGraphY);
          this.mapX = centerPct.x;
          this.mapY = centerPct.y;
        }

        if (this.currentPoseGraph) {
          this.currentPoseSvg = this.graphToSvg(this.currentPoseGraph.x, this.currentPoseGraph.y);
          this.currentPoseNodeId = this.findNearestNodeId(this.currentPoseGraph.x, this.currentPoseGraph.y);
          this.markCheckpointAsPassed(this.currentPoseNodeId);
        }

        this.updateMap();
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
    const overlay = this.overlayElementRef?.nativeElement ?? null;
    if (!overlay) {
      return;
    }

    if (!this.enableMapPan || !this.hasLocation) {
      overlay.style.top = `0%`;
      overlay.style.left = `0%`;
      overlay.style.width = `100%`;
      overlay.style.height = `100%`;
      return;
    }

    this.mapWidth = this.mapSize;
    this.mapHeight = 100;

    const top = (this.mapY * this.mapHeight) / 100 - this.mapHeight - (this.screenSize["height"] / 2 - this.mapHeight);
    const left = (this.mapX * this.mapWidth) / 100 - this.mapWidth - (this.screenSize["width"] / 2 - this.mapWidth);

    overlay.style.top = `${-top}%`;
    overlay.style.left = `${-left}%`;
    overlay.style.width = `${this.mapSize}%`;
    overlay.style.height = `${this.mapHeight}%`;
  }

  onSelectNode(nodeId: string): void {
    this.selectedNodeId = nodeId;
    this.webSocketService.sendMessageToFlask(
      `{\"Name\": \"GlobalPlanningGoalNodeId\", \"Value\": \"${nodeId}\"}`
    );
  }

  private graphToPercent(x: number, y: number): { x: number; y: number } {
    if (!this.graphBounds) {
      return {
        x: (x * 100) / 20.67,
        y: 100 - (y * 100) / 13.76
      };
    }
    const spanX = Math.max(0.0001, this.graphBounds.max_x - this.graphBounds.min_x);
    const spanY = Math.max(0.0001, this.graphBounds.max_y - this.graphBounds.min_y);

    return {
      x: ((x - this.graphBounds.min_x) * 100) / spanX,
      y: 100 - ((y - this.graphBounds.min_y) * 100) / spanY
    };
  }

  private graphToSvg(x: number, y: number): { x: number; y: number } {
    const imageSpanX = this.mapImageBounds.maxX - this.mapImageBounds.minX;
    const imageSpanY = this.mapImageBounds.maxY - this.mapImageBounds.minY;
    const padX = imageSpanX * this.mapFitPaddingRatio;
    const padY = imageSpanY * this.mapFitPaddingRatio;
    const minX = this.mapImageBounds.minX + padX;
    const maxX = this.mapImageBounds.maxX - padX;
    const minY = this.mapImageBounds.minY + padY;
    const maxY = this.mapImageBounds.maxY - padY;
    const fitSpanX = Math.max(0.0001, maxX - minX);
    const fitSpanY = Math.max(0.0001, maxY - minY);
    if (!this.graphBounds) {
      return {
        x: minX + (x / 20.67) * fitSpanX,
        y: minY + (1 - (y / 13.76)) * fitSpanY
      };
    }
    const spanX = Math.max(0.0001, this.graphBounds.max_x - this.graphBounds.min_x);
    const spanY = Math.max(0.0001, this.graphBounds.max_y - this.graphBounds.min_y);
    return {
      x: minX + ((x - this.graphBounds.min_x) / spanX) * fitSpanX,
      y: minY + (1 - ((y - this.graphBounds.min_y) / spanY)) * fitSpanY
    };
  }

  private findNearestNodeId(x: number, y: number): string | null {
    if (this.graphNodes.length === 0) {
      return null;
    }

    let nearestNodeId: string | null = null;
    let nearestDistSq = Number.POSITIVE_INFINITY;

    for (const node of this.graphNodes) {
      const dx = node.x - x;
      const dy = node.y - y;
      const distSq = dx * dx + dy * dy;
      if (distSq < nearestDistSq) {
        nearestDistSq = distSq;
        nearestNodeId = node.id;
      }
    }

    return nearestNodeId;
  }

  public isCheckpointNode(nodeId: string): boolean {
    return this.checkpointNodeIds.has(String(nodeId));
  }

  public isPassedCheckpointNode(nodeId: string): boolean {
    return this.passedCheckpointNodeIds.has(String(nodeId));
  }

  private markCheckpointAsPassed(nodeId: string | null): void {
    if (!nodeId) {
      return;
    }
    const key = String(nodeId);
    if (this.checkpointNodeIds.has(key)) {
      this.passedCheckpointNodeIds.add(key);
    }
  }
}
