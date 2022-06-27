namespace MySceneGraph

open System
open System.IO
open Aardvark.Base
open Aardvark.Rendering
open Aardvark.Rendering.Text
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
open Valve.VR

open VRVis.Vgm
open PointCloudSegmentation
open PointCloudSegmentation.DynamicSegmentation


    //let t =
    //    let m = M44d(        0.958058901193302,
    //        -0.286571355589288,
    //        0,
    //        -7311.5304327031,
    //        0.286571355589288,
    //        0.958058901193302,
    //        0,
    //        256905.212901514,
    //        0,
    //        0,
    //        1,
    //        706.748557267515,
    //        0,
    //        0,
    //        0,
    //        1)
    //    Trafo3d(m,m.Inverse)
    //let store = @"D:\jb_innen"
    //let key = 
    //    let k = Path.combine [(if System.IO.Directory.Exists store then store else System.IO.Path.GetDirectoryName store);"key.txt"] |> File.readAllText
    //    k.Trim([|' ';'\t';'\r';'\n'|])
    //let store = @"C:\Users\aszabo\bla\stores\3278_5511_0_10\pointcloud\data.bin"
    //let key = "3278_5511_0_10"
    //let tt (l : LodTreeInstance) = 
    //    let root = (l.root.Id :?> IPointCloudNode)
    //    let c = V3d root.CentroidLocal + root.Cell.GetCenter()
    //    let trafo = (Trafo3d.Translation -c)
    //    LodTreeInstance.transform trafo l


[<ReflectedDefinition>]
module Shader =
    open FShade
    open System.Runtime.CompilerServices
    
    type UniformScope with
        member x.Gamma : float = uniform?Gamma

    type Vertex = {
        [<Position>]                pos     : V4d
        [<Normal>]                  n       : V3d
        [<BiNormal>]                b       : V3d
        [<Tangent>]                 t       : V3d
        [<Color>]                   c       : V4d
        [<Semantic("LightDir")>]    ldir    : V3d
    }
    type Fragment = {
        [<Color>]   c       : V4d
        [<Depth>]   d       : float
    }

    [<ReflectedDefinition; Inline>]
    let div (v : V4d) = v.XYZ / v.W
    
    [<ReflectedDefinition>]
    let getNdcPointRadius (vp : V4d) (dist : float) =
        let ppx = uniform.ProjTrafo * (vp + V4d(0.5 * dist, 0.0, 0.0, 0.0))
        let ppy = uniform.ProjTrafo * (vp + V4d(0.0, 0.5 * dist, 0.0, 0.0))
        let ppz = uniform.ProjTrafo * vp

        let ppz = ppz.XYZ / ppz.W
        let d1 = ppx.XYZ / ppx.W - ppz |> Vec.length
        let d2 = ppy.XYZ / ppy.W - ppz |> Vec.length
        0.5 * (d1 + d2)


    [<GLSLIntrinsic("gl_FragCoord")>]
    let fragCoord() : V4d = onlyInShaderCode ""

    let depthOffset  (v : Vertex) =
        fragment {
            let fc = fragCoord()

            let ndc = 2.0 * (fc.XY / V2d uniform.ViewportSize) - V2d.II
            let pp = V4d(ndc.X, ndc.Y, fc.Z * 2.0 - 1.0, 1.0)

            let size = 0.09
            let vp = uniform.ProjTrafoInv * pp |> div
            let pp = uniform.ProjTrafo * V4d(vp.X, vp.Y, vp.Z + size, 1.0) |> div

            let depth = clamp 0.0 1.0 (pp.Z * 0.5 + 0.5)

            return { c = v.c; d = depth }
        }
    
    let onOffColor (v : PointSetShaders.PointVertex) =
        vertex {
            return { v with col = if uniform?ShowColors then v.col else V4d.IIII }
        }

    let env =
        samplerCube {
            texture uniform?EnvMap
            addressU WrapMode.Wrap
            addressV WrapMode.Wrap
            addressW WrapMode.Wrap
            filter Filter.MinMagMipLinear
        }

    let envMap (v : VRVis.Vgm.Shader.Vertex) =
        fragment {
            let vp = uniform.ProjTrafoInv * V4d(v.pos.X, v.pos.Y, -1.0, 1.0)
            let vp = vp.XYZ / vp.W
            let dir = (uniform.ViewTrafoInv * V4d(vp, 0.0)).XYZ |> Vec.normalize
            return env.Sample(dir)
        }

    let reverseTrafo (v : Effects.Vertex) =
        vertex {
            let wp = uniform.ViewProjTrafoInv * v.pos
            return { v with wp = wp / wp.W }
        }

    let gamma (v : VRVis.Vgm.Shader.Vertex) =
        fragment {
            return v.color ** (1.0 / uniform.Gamma)
        }


module SegmentationSceneGraph =
    
    let boundingBox =
        AdaptiveReduction.halfGroup
            Box3d.Invalid
            (fun l r -> Box.Union(l,r))
            (fun l r -> if l.Min.AnyEqual r.Min || l.Max.AnyEqual r.Max then ValueNone else ValueSome l)

    let scene (res : aval<Option<SegmentationResult>>) (world2Local : aval<Trafo3d>) (view : aval<Trafo3d>) (vps : aval<V2i>) =
        

        let largestr (fmt : string) (unit : string) (value : float) =
            let str (a : float) = a.ToString(fmt)
            let a = abs value
            if Fun.IsTiny a then "0"
            elif a > 100_000_000.0 then sprintf "%sG%s" (str (value / 1000_000_000.0)) unit
            elif a > 100_000.0 then sprintf "%sM%s" (str (value / 1000_000.0)) unit
            elif a > 100.0 then sprintf "%sk%s" (str (value / 1000.0)) unit
            elif a > 0.1 then sprintf "%s%s" (str value) unit
            elif a > 0.01 then sprintf "%sc%s" (str (100.0 * value)) unit
            elif a > 0.0001 then sprintf "%sm%s" (str (1000.0 * value)) unit
            elif a > 0.0000001 then sprintf "%su%s" (str (1000000.0 * value)) unit
            elif a > 0.0000000001 then sprintf "%sn%s" (str (1000000000.0 * value)) unit
            else sprintf "%sp%s" (str (1000000000000.0 * value)) unit
            
        let largeInt (fmt : string) (unit : string) (value : int) =
            let str (a : float) = a.ToString(fmt)
            let a = abs value
            if a = 0 then "0"
            elif a > 1_000_000_000 then sprintf "%sG%s" (str (float value / 1_000_000_000.0)) unit
            elif a > 1_000_000 then sprintf "%sM%s" (str (float value / 1_000_000.0)) unit
            elif a > 1000 then sprintf "%sk%s" (str (float value / 1000.0)) unit
            else sprintf "%d%s" value unit
            
        let largeSqrstr (unit : string) (value : float) =
            let a = abs value
            if Fun.IsTiny a then "0"
            elif a > 100_000.0 then sprintf "%.2fk%s²" (value / 1000_000.0) unit
            elif a > 0.1 then sprintf "%.2f%s²" value unit
            elif a > 0.00001 then sprintf "%.2fc%s²" (10000.0 * value) unit
            elif a > 0.0000001 then sprintf "%.2fm%s²" (1000000.0 * value) unit
            else sprintf "%.2fu%s²" (1_000_000_000_000.0 * value) unit
            
        let config =
            {
                font = FontSquirrel.Hack.Regular
                color = C4b.White
                align = TextAlignment.Center
                flipViewDependent = true
                renderStyle = RenderStyle.Normal
            }

        let picks =
            let data = 
                (res, world2Local) ||> AVal.map2 (fun seg w2l ->
                    match seg with
                    | Some res ->
                        let w2l = w2l.Inverse
                        let vertices = res.Planes |> Array.sumBy (fun pi -> pi.vertices.Length)
                        let positions = Array.zeroCreate vertices
                        let lineIndex = res.Planes |> Array.sumBy (fun pi -> pi.outline.Length) |> Array.zeroCreate 
                        let triangleIndex = res.Planes |> Array.sumBy (fun pi -> pi.interior.Length) |> Array.zeroCreate 

                        let mutable offset = 0
                        let mutable vi = 0
                        let mutable li = 0
                        let mutable ti = 0
                        for p in res.Planes do
                            for v in p.vertices do
                                positions.[vi] <- V3f (w2l.Forward.TransformPos v)
                                vi <- vi + 1

                            for i in p.outline do
                                lineIndex.[li] <- offset + i
                                li <- li + 1

                            for i in p.interior do
                                triangleIndex.[ti] <- offset + i
                                ti <- ti + 1

                            offset <- vi

                        let lineTrafos =
                            res.Lines |> Array.map (fun li ->
                                li.trafo *
                                w2l
                                //let p0 = li.line.P0
                                //let p1 = li.line.P1

                                //let d = p1 - p0
                                //let l = Vec.length d

                                //Trafo3d.Scale(0.01, 0.01, l) *
                                //Trafo3d.FromNormalFrame(p0, d / l) *
                                //ct
                            )

                        let sphereTrafos =
                            res.Points |> Array.map (fun pi ->
                                Trafo3d.Scale(0.03) *
                                Trafo3d.Translation(pi.position) *
                                w2l
                            
                            )
                            
                        let pointTrafos =
                            res.Points |> Array.map (fun pi ->
                                Trafo3d.Scale(10.0) *
                                pi.trafo *
                                w2l
                            
                            )
                            

                        let texts =
                            res.Planes |> Array.map (fun pi ->
                                //let cc = pi.vertices.ComputeCentroid()

                                let mutable sum = V3d.Zero
                                let mutable wsum = 0.0
                                for i0 in 0 .. 3 .. pi.interior.Length - 3 do
                                    let i1 = i0 + 1
                                    let i2 = i0 + 2

                                    let p0 = pi.vertices.[pi.interior.[i0]]
                                    let p1 = pi.vertices.[pi.interior.[i1]]
                                    let p2 = pi.vertices.[pi.interior.[i2]]

                                    let tri = Triangle3d(p0, p1, p2)
                                    let c = tri.ComputeCentroid()
                                    let w = tri.Area
                                    sum <- sum + w * c
                                    wsum <- wsum + w

                                let cc = sum / wsum

                                let text =  
                                    sprintf "%s\r\n±%s\r\n%s" 
                                        (largeInt "0.0" "pts" pi.pointCount) 
                                        (largestr "0.00" "m" pi.rmse) 
                                        (largeSqrstr "m" wsum)
                                let shape = config.Layout(text)


                                let c = shape.bounds.Center
                                let centerTrafo = Trafo3d.Translation(0.0, -c.Y, 0.0)

                                //let outline = ConcreteShape.fillRoundedRectangle (C4b(255uy,0uy,0uy,100uy)) 0.3 (shape.bounds.EnlargedBy 0.3)

                                

                                //let shape = shape |> ShapeList.prepend outline


                                let size = (min pi.sizes.X pi.sizes.Y) * 0.07
                                let trafo =
                                    let z = pi.plane.Normal |> Vec.normalize
                                        
                                    let a = Constant.DegreesPerRadian * acos (z.Z |> abs)

                                    if a < 10.0 then
                                        let x = pi.trafo.Forward.C0.XYZ |> Vec.normalize
                                        let r = (AVal.force view).Backward.C0.XYZ |> Vec.normalize
                                        let x = if Vec.dot x r > 0.0 then -x else x


                                        let y = Vec.cross z x |> Vec.normalize
                                        let t = 
                                            centerTrafo *
                                            Trafo3d.Scale(size) * 
                                            Trafo3d.FromBasis(x, y, z, cc) * 
                                            w2l

                                        t
                                    else
                                        let x = Vec.cross V3d.OOI z |> Vec.normalize
                                        let y = Vec.cross z x |> Vec.normalize
                                        let t = 
                                            centerTrafo *
                                            Trafo3d.Scale(size) * 
                                            Trafo3d.FromBasis(x, y, z, cc) * 
                                            w2l
                                        t

                                

                                let trafo = 
                                    view |> AVal.map (fun view ->
                                        
                                        let z = pi.plane.Normal |> Vec.normalize
                                        let cam = view.Backward.C3.XYZ
                                        let center = pi.centroid
                                        let fw = center - cam |> Vec.normalize




                                        let z = 
                                            if Vec.dot z fw > 0.0 then -z
                                            else z



                                        ////let vp = view * proj
                                        //let center = view.Forward.TransformPos trafo.Forward.C3.XYZ


                                        //let bla = Trafo3d.Translation(0.0, 0.0, 0.15) //Trafo3d.Scale 1.5

                                        



                                        trafo *
                                        Trafo3d.Translation(z * 0.1)
                                        //view *
                                        //bla *
                                        //view.Inverse

                                    )


                                
                                trafo, AVal.constant shape
                            )

                        lineTrafos, sphereTrafos, positions, lineIndex, triangleIndex, texts, pointTrafos
                    | None ->
                        [||], [||], [||], [||], [||], [||], [||]
                )


            let pass0 = RenderPass.after "Segmentation" RenderPassOrder.Arbitrary RenderPass.main
            let pass1 = RenderPass.after "Segmentation2" RenderPassOrder.Arbitrary pass0

            Sg.ofList [
                Sg.ofList [
                    Aardvark.SceneGraph.SgPrimitives.Sg.cylinder' 10 C4b.Red 2.0 1.0
                    |> Sg.instanced (AVal.map (fun (a,_,_,_,_,_,_) -> a) data)
                    |> Sg.shader {
                        do! DefaultSurfaces.stableTrafo
                        do! DefaultSurfaces.constantColor C4f.Red
                        do! DefaultSurfaces.stableHeadlight
                        do! Shader.depthOffset
                    }

                    
                    Aardvark.SceneGraph.SgPrimitives.Sg.box' C4b.Red (Box3d(-V3d.III, V3d.III))
                    |> Sg.instanced (AVal.map (fun (_,_,_,_,_,_,a) -> a) data)
                    |> Sg.shader {
                        do! DefaultSurfaces.stableTrafo
                        do! DefaultSurfaces.constantColor C4f.Blue
                        do! DefaultSurfaces.stableHeadlight
                        do! Shader.depthOffset
                    }

                ]
                |> Sg.pass pass0

                //Aardvark.SceneGraph.SgPrimitives.Sg.sphere' 5 C4b.Red 1.0
                //|> Sg.instanced (AVal.map (fun (_,a,_,_,_,_,_) -> a) data)
                //|> Sg.shader {
                //    do! Shader.depthOffset 0.1
                //    do! DefaultSurfaces.constantColor C4f.Blue
                //    do! DefaultSurfaces.stableHeadlight
                //}
                    
                Sg.ofList [
                    Sg.draw IndexedGeometryMode.LineList
                    |> Sg.index (AVal.map (fun (_,_,_,a,_,_,_) -> a) data)
                    |> Sg.shader {
                        do! DefaultSurfaces.stableTrafo
                        do! DefaultSurfaces.thickLine
                        do! DefaultSurfaces.constantColor C4f.Yellow
                        do! DefaultSurfaces.thickLineRoundCaps
                        do! Shader.depthOffset
                    }
                    |> Sg.uniform "LineWidth" (AVal.constant 2.0)

                    Sg.draw IndexedGeometryMode.TriangleList
                    |> Sg.index (AVal.map (fun (_,_,_,_,a,_,_) -> a) data)
                    |> Sg.shader {
                        do! DefaultSurfaces.stableTrafo
                        do! Shader.depthOffset
                        //do! DefaultSurfaces.sgColor (C4f(0.0f, 1.0f, 0.0f, 0.1f))
                    }
                    |> Sg.vertexBufferValue DefaultSemantic.Colors (AVal.constant (V4f(0.0f, 1.0f, 0.0f, 0.1f)))
                    |> Sg.fillMode (AVal.constant (FillMode.Fill))
                
                ]
                |> Sg.vertexAttribute DefaultSemantic.Positions (AVal.map (fun (_,_,a,_,_,_,_) -> a) data)
                


                data 
                |> AVal.map (fun (_,_,_,_,_,a,_) -> HashSet.ofArray a)
                |> ASet.ofAVal
                |> Sg.shapes
                |> Sg.uniform "DepthBias" (AVal.constant (0.0001))

            
            ]
            |> Sg.depthTest (AVal.constant DepthTest.LessOrEqual)
            |> Sg.blendMode (AVal.constant BlendMode.Blend)
            |> Sg.pass pass1

        let sg =
            Sg.ofList [
                picks
                //Skybox.skyboxSg "miramar_$.png" 
            ]
            |> Sg.uniform "ViewportSize" vps

        sg