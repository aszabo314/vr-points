namespace MySceneGraph

open System
open Aardvark.Base
open Aardvark.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim
open FSharp.Data.Adaptive
open FSharp.Data.Adaptive.Operators
open Aardvark.Application.Utilities
open Aardvark.Rendering.PointSet
open Aardvark.Base.Ag
open Aardvark.Geometry.Points
open Aardvark.Data.Points
open Aardvark.Data.Points.Import
open MySceneGraph
open FShade



[<ReflectedDefinition>]
module PointSetShaders =
    open FShade
    open Aardvark.Rendering


    let constantColor (c : C4f) (v : Effects.Vertex) =
        let c = c.ToV4d()
        vertex {
            return { v with c = c }
        }


    let heatMapColors =
        let fromInt (i : int) =
            C4b(
                byte ((i >>> 16) &&& 0xFF),
                byte ((i >>> 8) &&& 0xFF),
                byte (i &&& 0xFF),
                255uy
            ).ToC4f().ToV4d()

        Array.map fromInt [|
            0x1639fa
            0x2050fa
            0x3275fb
            0x459afa
            0x55bdfb
            0x67e1fc
            0x72f9f4
            0x72f8d3
            0x72f7ad
            0x71f787
            0x71f55f
            0x70f538
            0x74f530
            0x86f631
            0x9ff633
            0xbbf735
            0xd9f938
            0xf7fa3b
            0xfae238
            0xf4be31
            0xf29c2d
            0xee7627
            0xec5223
            0xeb3b22
        |]

    let heat (tc : float) =
        let tc = clamp 0.0 1.0 tc
        let fid = tc * float heatMapColors.Length - 0.5

        let id = int (floor fid)
        if id < 0 then 
            heatMapColors.[0]
        elif id >= heatMapColors.Length - 1 then
            heatMapColors.[heatMapColors.Length - 1]
        else
            let c0 = heatMapColors.[id]
            let c1 = heatMapColors.[id + 1]
            let t = fid - float id
            (c0 * (1.0 - t) + c1 * t)


    type UniformScope with
        member x.PointVisualization : PointVisualization = x?PointVisualization
        member x.Overlay : V4d[] = x?StorageBuffer?Overlay
        member x.ModelTrafos : M44d[] = x?StorageBuffer?ModelTrafos
        member x.LeftModelViewTrafos : M44d[] = x?StorageBuffer?LeftModelViewTrafos
        member x.RightModelViewTrafos : M44d[] = x?StorageBuffer?RightModelViewTrafos
        member x.Scales : V4d[] = x?StorageBuffer?Scales

        member x.NearFar : V2d = uniform?NearFar
        member x.PlaneFit : bool = uniform?PlaneFit
        member x.PlaneFitTolerance : float = uniform?PlaneFitTolerance // 0.05
        member x.PlaneFitRadius : float = uniform?PlaneFitRadius // 7.0
        

    type Vertex =
        {
            [<Position>] pos : V4d
            [<Normal>] n : V3d
            [<Semantic("Offsets")>] offset : V3d
        }


    let offset ( v : Vertex) =
        vertex {
            return  { v with pos = v.pos + V4d(v.offset, 0.0)}
        }
        
    
    type PointVertex =
        {
            [<Position>] pos : V4d
            [<Color; Interpolation(InterpolationMode.Flat)>] col : V4d
            //[<Normal>] n : V3d
            [<Semantic("ViewCenter"); Interpolation(InterpolationMode.Flat)>] vc : V3d
            [<Semantic("ViewPosition")>] vp : V3d
            [<Semantic("AvgPointDistance")>] dist : float
            [<Semantic("DepthRange"); Interpolation(InterpolationMode.Flat)>] depthRange : float
            [<PointSize>] s : float
            [<Semantic("PointPixelSize")>] ps : float
            [<PointCoord>] c : V2d
            [<Normal>] n : V3d
            [<Semantic("TreeId")>] id : int
            [<Semantic("MaxTreeDepth")>] treeDepth : int
            [<FragCoord>] fc : V4d
            [<SamplePosition>] sp : V2d
        }



    let getNdcPointRadius (vp : V4d) (dist : float) =
        let ppx = uniform.ProjTrafo * (vp + V4d(0.5 * dist, 0.0, 0.0, 0.0))
        let ppy = uniform.ProjTrafo * (vp + V4d(0.0, 0.5 * dist, 0.0, 0.0))
        let ppz = uniform.ProjTrafo * vp

        let ppz = ppz.XYZ / ppz.W
        let d1 = ppx.XYZ / ppx.W - ppz |> Vec.length
        let d2 = ppy.XYZ / ppy.W - ppz |> Vec.length
        0.5 * (d1 + d2)
        
    let div (v : V4d) = v.XYZ / v.W



    [<GLSLIntrinsic("gl_InvocationID")>]
    let idx() = onlyInShaderCode ""
    
    [<GLSLIntrinsic("gl_Layer")>]
    let layer() : int = onlyInShaderCode ""

    let lodPointSize2 (v : PointVertex) =
        vertex { 
            let mv = 
                if idx() = 0 then uniform.LeftModelViewTrafos.[v.id]
                else uniform.RightModelViewTrafos.[v.id]

            let vp = mv * v.pos
            let vn = mv * V4d(v.n, 0.0) |> Vec.xyz
            let pp = div (uniform.ProjTrafo * vp)

            let pixelSize = uniform.PointSize
            let ndcRadius = pixelSize / V2d uniform.ViewportSize

            let vpx = uniform.ProjTrafoInv * (V4d(pp.X + ndcRadius.X, pp.Y, pp.Z, 1.0)) |> div
            let vpy = uniform.ProjTrafoInv * (V4d(pp.X, pp.Y + ndcRadius.Y, pp.Z, 1.0)) |> div
            let dist = 0.5 * (Vec.length (vpx - vp.XYZ) + Vec.length (vpy - vp.XYZ))

            let vpz = vp + V4d(0.0, 0.0, dist, 0.0)
            let fpp = uniform.ProjTrafo * vpz
            let opp = uniform.ProjTrafo * vp
        
            let pp0 = opp.XYZ / opp.W
            let ppz = fpp.XYZ / fpp.W
        
            let depthRange = abs (pp0.Z - ppz.Z)

            let pixelSize = 
                if ppz.Z < -1.0 then -1.0
                else pixelSize
            
            let col =
                if uniform.PointVisualization &&& PointVisualization.Color <> PointVisualization.None then
                    v.col.XYZ
                else
                    V3d.III

            return { v with ps = pixelSize; n = vn; s = pixelSize; pos = opp; depthRange = depthRange; vp = vp.XYZ; vc = vp.XYZ; col = V4d(col, v.col.W) }
        }

    let color =
        sampler2dArray {
            texture uniform?DiffuseColorTexture
        }
    let depthTex =
        sampler2dArray {
            texture uniform?DepthTexture
        }

    

    let compose (v : Effects.Vertex) =
        fragment { 
            let color = color.SampleLevel(v.tc, layer(), 0.0)
            let ndc = v.pos.XY / v.pos.W

            let invSize = 1.0 / V2d uniform.ViewportSize

            let d = depthTex.SampleLevel(v.tc, layer(), 0.0).X * 2.0 - 1.0
            let dx = depthTex.SampleLevel(v.tc + V2d(invSize.X, 0.0), layer(), 0.0).X * 2.0 - 1.0
            let dy = depthTex.SampleLevel(v.tc + V2d(0.0, invSize.Y), layer(), 0.0).X * 2.0 - 1.0

            let vp0 = div (uniform.ProjTrafoInv * V4d(ndc, d, 1.0))
            let vpx = div (uniform.ProjTrafoInv * V4d(ndc + V2d(2.0 * invSize.X, 0.0), dx, 1.0))
            let vpy = div (uniform.ProjTrafoInv * V4d(ndc + V2d(0.0, -2.0 * invSize.Y), dy, 1.0))

            let u = vpx - vp0
            let v = vpy - vp0
            let n = Vec.cross u v |> Vec.normalize


            let d = Vec.dot n (Vec.normalize vp0)


            return V4d(color.XYZ * clamp 0.0 1.0 d, 1.0) //V4d(0.5 * n + V3d.Half, 0.9) //V4d(V3d.III * d ** 32.0, 1.0)
        }

    type Fragment =
        {
            [<Color>] c : V4d
            [<Depth>] d : float
        }

    let lodPointCircular (v : PointVertex) =
        fragment {
            let c = v.c * 2.0 - V2d.II
            let f = Vec.dot c c - 1.0
            if f > 0.0 then discard()
            
            let t = sqrt -f
            let depth = v.fc.Z
            let outDepth = depth - v.depthRange * t
            
            let alpha = v.col.W
            let mutable color = v.col.XYZ

            if uniform.PointVisualization &&& PointVisualization.Lighting <> PointVisualization.None then
                let f = Vec.dot c c
                let diffuse = sqrt (max 0.0 (1.0 - f))
                color <- color * diffuse

            return { c = V4d(color, alpha); d = outDepth }
        }




    let cSam =
        sampler2dArray {
            texture uniform?Colors
            filter Filter.MinMagPoint
        }
        
    let depthSam =
        sampler2dArray {
            texture uniform?DepthStencil
            filter Filter.MinMagPoint
        }
    
    let randomSam =
        sampler2d {
            texture uniform?RandomTexture
            filter Filter.MinMagPoint
        }

    [<Inline>]
    let sampleDepth (tc : V2d) =
        depthSam.SampleLevel(tc, layer(), 0.0).X * 2.0 - 1.0

    [<Inline>]
    let viewPosSize() =
        V2d depthSam.Size

    let sampleViewPos (tc : V2d) =
        let z = sampleDepth tc
        if z >= 0.99999 then
            V3d.Zero
        else
            let ndc = 2.0 * tc - 1.0
            let vp = uniform.ProjTrafoInv * V4d(ndc, z, 1.0)
            vp.XYZ / vp.W

    [<Inline>]
    let sampleSimpleNormal (vp : V3d) (tc : V2d) =
        let s = viewPosSize()
        let vpx = sampleViewPos(tc + V2d.IO / s)
        let vpy = sampleViewPos(tc + V2d.OI / s)
        let vnx = sampleViewPos(tc - V2d.IO / s)
        let vny = sampleViewPos(tc - V2d.OI / s)
            
        let z = abs vp.Z < 0.0001
        let zx = abs vpx.Z < 0.0001
        let zy = abs vpy.Z < 0.0001
        let nx = abs vpx.Z < 0.0001
        let ny = abs vpy.Z < 0.0001

        if z || (zx && nx) || (zy && ny) then 
            -Vec.normalize vp
        //elif zx || zy || z || abs(vp.Z - vpx.Z) > 0.1 || abs(vp.Z - vpy.Z) > 0.1 then
        //    V3d.Zero
        elif not zx && not zy then
            let n = Vec.cross (vpx - vp) (vpy - vp)
            Vec.normalize n
        elif not zx && not ny then
            let n = Vec.cross (vpx - vp) (vp - vny)
            Vec.normalize n
        elif not nx && not zy then
            let n = Vec.cross (vp - vnx) (vpy - vp)
            Vec.normalize n
        else
            let n = Vec.cross (vp - vnx) (vp - vny)
            Vec.normalize n
  
    let realRootsOfNormed (c2 : float) (c1 : float) (c0 : float) =
        let mutable d = c2 * c2
        let p3 = 1.0/3.0 * (-1.0/3.0 * d + c1)
        let q2 = 1.0/2.0 * ((2.0/27.0 * d - 1.0/3.0 * c1) * c2 + c0)
        let p3c = p3 * p3 * p3
        let shift = 1.0/3.0 * c2
        d <- q2 * q2 + p3c
        if d < 0.0 then
            if p3c > 0.0 || p3 > 0.0 then
                -1.0
            else
                let v = -q2 / sqrt(-p3c)
                if v < -1.0 || v > 1.0 then
                    -1.0
                else
                    let phi = 1.0 / 3.0 * acos v
                    let t = 2.0 * sqrt(-p3)
                    let r0 = t * cos phi - shift
                    let r1 = -t * cos (phi + Constant.Pi / 3.0) - shift
                    let r2 = -t * cos (phi - Constant.Pi / 3.0) - shift
                    min r0 (min r1 r2)
        
        else
            d <- sqrt d
            let uav = cbrt (d - q2) - cbrt (d + q2)
            let s0 = uav - shift
            let s1 = -0.5 * uav - shift
            min s0 s1
  




    type Fraggy =
        {
            [<Color>]
            color : V4d
            
            [<Normal>]
            normal : V3d

            [<Depth>]
            depth : float
        }
    
    let samples24 =
        [|
            //V2d( 0.0, 0.0 )
            V2d( -0.4612850228120782, -0.8824263018037591 )
            V2d( 0.2033539719528926, 0.9766070232577696 )
            V2d( 0.8622755945065503, -0.4990552917715807 )
            V2d( -0.8458406529500018, 0.4340626564690164 )
            V2d( 0.9145341241356336, 0.40187426079092753 )
            V2d( -0.8095919285224212, -0.2476471278659192 )
            V2d( 0.2443597793708885, -0.8210571365841042 )
            V2d( -0.29522102954593127, 0.6411496844366571 )
            V2d( 0.4013698454531175, 0.47134750051312063 )
            V2d( -0.1573158341083741, -0.48548502348882533 )
            V2d( 0.5674301785250454, -0.1052346781436156 )
            V2d( -0.4929375319230899, 0.09422383038685558 )
            V2d( 0.967785465127825, -0.06868225365333279 )
            V2d( 0.2267967507441493, -0.40237871966279687 )
            V2d( -0.7200979001122771, -0.6248240905561527 )
            V2d( -0.015195608523765971, 0.35623701723070667 )
            V2d( -0.11428925675805125, -0.963723441683084 )
            V2d( 0.5482105069441386, 0.781847612911249 )
            V2d( -0.6515264455787967, 0.7473765703131305 )
            V2d( 0.5826875031269089, -0.6956573112908789 )
            V2d( -0.8496230198638387, 0.09209564840857346 )
            V2d( 0.38289808661249414, 0.15269522898022844 )
            V2d( -0.4951171173546325, -0.2654758742352245 )
        |]



    let sampleNormal (vp : V3d) (tc : V2d) =
        let nf = uniform.NearFar
        let ld = -vp.Z

        if ld > 0.0 && ld < nf.Y && uniform.PlaneFit then
            let vn = sampleSimpleNormal vp tc 
            if vn = V3d.Zero then   
                V4d(V3d.OOI, nf.Y + 10.0)
            else
                let size = viewPosSize()
                //let id0 = pidSam.SampleLevel(tc, 0.0).X |> abs |> round |> int
                //let mutable id1 = -1
                //let mutable id2 = -1

                let plane = V4d(vn, -Vec.dot vn vp)

                let mutable sum = V3d.Zero
                let mutable sumSq = V3d.Zero
                let mutable off = V3d.Zero
                let mutable cnt = 1

                let x = randomSam.SampleLevel((floor (tc * viewPosSize()) + V2d.Half) / V2d randomSam.Size, 0.0).XY |> Vec.normalize
                let y = V2d(-x.Y, x.X)

                for o in samples24 do
                    let tc = tc + uniform.PlaneFitRadius * (x*o.X + y*o.Y) / size
                    let p = sampleViewPos tc
                    
                    if p.Z <> 0.0 && abs (Vec.dot plane (V4d(p, 1.0))) <= uniform.PlaneFitTolerance then
                        
                        //if id1 < 0 then
                        //    let o = pidSam.SampleLevel(tc, 0.0).X |> abs |> round |> int
                        //    if o <> id0 then id1 <- o
                        //elif id2 < 0 then
                        //    let o = pidSam.SampleLevel(tc, 0.0).X |> abs |> round |> int
                        //    if o <> id0 && o <> id1 then id2 <- o
                            

                        let pt = p - vp
                        sum <- sum + pt
                        sumSq <- sumSq + sqr pt
                        off <- off + V3d(pt.Y*pt.Z, pt.X*pt.Z, pt.X*pt.Y)
                        cnt <- cnt + 1

                if cnt >= 8 then

                    let n = float cnt
                    let avg = sum / n
                    let xx = (sumSq.X - avg.X * sum.X) / (n - 1.0)
                    let yy = (sumSq.Y - avg.Y * sum.Y) / (n - 1.0)
                    let zz = (sumSq.Z - avg.Z * sum.Z) / (n - 1.0)

                    let xy = (off.Z - avg.X * sum.Y) / (n - 1.0)
                    let xz = (off.Y - avg.X * sum.Z) / (n - 1.0)
                    let yz = (off.X - avg.Y * sum.Z) / (n - 1.0)
            
                    let _a = 1.0
                    let b = -xx - yy - zz
                    let c = -sqr xy - sqr xz - sqr yz + xx*yy + xx*zz + yy*zz
                    let d = -xx*yy*zz - 2.0*xy*xz*yz + sqr xz*yy + sqr xy*zz + sqr yz*xx


                    let l = realRootsOfNormed b c d
                    if l < 0.0 then
                        V4d(vn, -Vec.dot vn vp)
                    else
                        let c0 = V3d(xx - l, xy, xz)
                        let c1 = V3d(xy, yy - l, yz)
                        let c2 = V3d(xz, yz, zz - l)
                        let len0 = Vec.lengthSquared c0
                        let len1 = Vec.lengthSquared c1
                        let len2 = Vec.lengthSquared c2

                        let normal =
                            if len0 > len1 then
                                if len2 > len1 then Vec.cross c0 c2
                                else Vec.cross c0 c1
                            else
                                if len2 > len0 then Vec.cross c1 c2
                                else Vec.cross c0 c1

                        let len = Vec.length normal

                        if len > 0.0 then
                            let normal = 
                                if normal.Z < 0.0 then -normal / len
                                else normal / len

                            V4d(normal, -Vec.dot normal (vp + avg))
                        else
                            V4d(vn, -Vec.dot vn vp)
                else
                    V4d(vn, -Vec.dot vn vp)


        else
            let vn = sampleSimpleNormal vp tc
            V4d(vn, -Vec.dot vn vp)


    let blitPlaneFit (v : Effects.Vertex) =
        fragment {
            let z = sampleDepth v.tc
            if z < -1.0 || z >= 1.0 then discard()
            let vp = sampleViewPos v.tc

            let c = cSam.SampleLevel(v.tc, layer(), 0.0).XYZ
            let plane = sampleNormal vp v.tc
            let n = plane.XYZ

            let diffuse = 
                if uniform?Diffuse then (Vec.dot (Vec.normalize vp.XYZ) n) |> abs
                else 1.0

            let col = c.XYZ * (0.2 + 0.8*diffuse)

            let mutable finalDepth = 2.0

            if n.Z <> 0.0 then
                // l*<vp|n> + w = 0
                let l = -plane.W / Vec.dot vp.XYZ n
                let npos = vp.XYZ * l
                let pp = uniform.ProjTrafo * V4d(npos, 1.0)
                finalDepth <- pp.Z / pp.W
            else
                finalDepth <- sampleDepth v.tc


            //let id = pidSam.SampleLevel(v.tc, 0.0).X |> round |> abs |> int
            //let pixel = V2i(id % uniform.ViewportSize.X, id / uniform.ViewportSize.X)
            //let tc = (V2d pixel + V2d.Half) / V2d uniform.ViewportSize

            return {
                color = V4d(col, 1.0)
                normal = n
                depth = finalDepth * 0.5 + 0.5
            }
        }
